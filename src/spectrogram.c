/*
 * spectrum.c
 * 
 * The spectrum is formed by dividing the input into overlapping segments, 
 * multiplying the segment by a window, performing an FFT, and then
 * averaging the magnitude of the FFT.
 *
 * -------------------------------------------------------------------------------- 
 * THIS SOFTWARE IS PROVIDED BY EOS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL EOS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Embedded Ocean Systems (EOS), 2019
 *
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <compiler.h>
#include <math.h>

#include "rtc_time.h"
#include "wispr.h"
#include "spectrogram.h"

//
// statically allocated data arrays so avoid runtime allocation
//
COMPILER_WORD_ALIGNED uint8_t psd_fft_input[4*PSD_MAX_FFT_SIZE];
COMPILER_WORD_ALIGNED uint8_t psd_fft_output[4*PSD_MAX_FFT_SIZE];

// FFT window
COMPILER_WORD_ALIGNED uint8_t psd_fft_window[4*PSD_MAX_FFT_SIZE];

arm_rfft_instance_q31 psd_twid_q31;	// pre-calculated twiddle factors
arm_rfft_fast_instance_f32 psd_twid_f32;	// pre-calculated twiddle factors

uint16_t psd_fft_size;
uint16_t psd_fft_overlap;
uint16_t psd_num_freq_bins;  // nfft/2 +1
uint32_t psd_sampling_freq; // samples per second
uint32_t psd_freq_bin_size; // hz/bin
uint8_t psd_sample_size; // samples per second

// local instance of the wispr data header to use for updating the data buffer
static wispr_data_header_t psd_data_header;

// 
// window function based on the matlab function
//
void spectrum_window(float32_t *w, uint8_t type, uint16_t size)
{
	float32_t a0, a1, a2, a3, a4;
	
	// Default Hamming window
	a0 = 0.54; a1 = 0.46; a2 = 0; a3 = 0; a4 = 0;
	
	switch (type) 
	{
		case HANN_WINDOW:
		// Hann window
			a0 = 0.5; a1 = 0.5; a2 = 0; a3 = 0; a4 = 0;
			break;
		case HAMMING_WINDOW:
		// Hamming window
			a0 = 0.54; a1 = 0.46; a2 = 0; a3 = 0; a4 = 0;
			break;
		case BLACKMAN_WINDOW: 
		// Blackman window 
			a0 = 0.42; a1 = 0.5; a2 = 0.08; a3 = 0; a4 = 0;
			break;
	}

	//x = (0:m-1)'/(n-1);
	int half = size/2;
	for(int m = 0; m < half; m++) {
		float32_t x = (float32_t)m / (float32_t)size;
		w[m] = a0 - a1*arm_cos_f32(2*M_PI*x) + a2*arm_cos_f32(4*M_PI*x) - a3*arm_cos_f32(6*M_PI*x) + a4*arm_cos_f32(8*M_PI*x);
		w[size-m-1] = w[m];
	}
	
	//for(int m = 0; m < size; m++) printf("%f ", w[m]);
	//printf("\r\n");
	
}

int spectrum_init_f32(wispr_config_t *wispr, uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint8_t bps, uint8_t wintype)
{
	// check spectrum size
	if( *nbins > nfft/2 ) {
		*nbins = nfft/2;
		//printf("spectrum_init_f32: number of bins greater than half fft size: %d\r\n", nbins);
	}

	// check number of bins
	if(*nbins > PSD_MAX_BINS_PER_BUFFER) {
		*nbins = PSD_MAX_BINS_PER_BUFFER;
		printf("spectrum_init_f32: number of bin truncated to %d\r\n", *nbins);
	}
	psd_num_freq_bins = *nbins;

	// check fft size
	if( nfft > PSD_MAX_FFT_SIZE ) {
		printf("spectrum_init_f32: unsupported fft size %d\r\n", nfft);
		return(ARM_MATH_LENGTH_ERROR);
	}	
	psd_fft_size = nfft;
	
	// check overlap
	if( overlap >= psd_fft_size || overlap < 0  ) {
		printf("spectrum_init_f32: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_fft_overlap = overlap;
	
	// check bytes per sample
	if( (bps < 2) || (bps > 4)  ) {
		printf("spectrum_init_f32: unsupported sample size %d\r\n", bps);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_sample_size = bps;

	// pre-calculated twiddle factors
	arm_status status = arm_rfft_fast_init_f32(&psd_twid_f32, psd_fft_size);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window((float32_t *)psd_fft_window, wintype, psd_fft_size);

	// max adc value used for scaling
	float32_t max_value  = 2147483648.0; // 2^31
	if( psd_sample_size == 3) {
		max_value = 8388608.0; // 2^23
	}
	else if( psd_sample_size == 2 ) {
		max_value = 32768.0; // 2^15
	}

	float32_t scale = (ADC_VREF/2.0) / max_value;
	float32_t *win_f32 = (float32_t *)psd_fft_window;
	for(int n = 0; n < psd_fft_size; n++) {
		win_f32[n] *= scale;
	}
	
	// update local data header structure with the current config
	// this is used to update the current data buffer header
	//wispr_update_data_header(wispr, &psd_data_header);
	
	psd_sampling_freq = wispr->sampling_rate; // samples per second (hz)
	psd_freq_bin_size = 1000 * psd_sampling_freq / psd_fft_size; // 1000 * hz/bin

	// update local data header structure with the current config
	wispr_update_data_header(wispr, &psd_data_header);
	psd_data_header.settings[0] = WISPR_SPECTRUM;

	// but change settings, sampling rate (1000*hz/bin), sample_size, ...
	psd_data_header.sampling_rate = psd_freq_bin_size;
	psd_data_header.sample_size = 4;
	psd_data_header.samples_per_block = psd_num_freq_bins;
	psd_data_header.block_size = PSD_MAX_BUFFER_SIZE; // number of bytes in an psd record block
	
	printf("spectrum_init_f32:  nfft=%d, nbins=%d, overlap=%d, bps=%d\r\n", 
		psd_fft_size, psd_num_freq_bins, psd_fft_overlap, psd_sample_size);

	return(status);
}

//
// Calculate the spectrum for the input signal
// using a floating point (float32) real fft.
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//
//int spectrum_f32(float32_t *output, uint8_t *input, uint16_t nsamps, int bps, uint16_t overlap, uint8_t *hdr)
int spectrum_f32(uint8_t *psd_buffer, uint8_t *adc_buffer, uint16_t nsamps)
{
	int k, m, n;
	float32_t *win = (float32_t *)psd_fft_window;
	float32_t *obuf = (float32_t *)psd_fft_output;
	float32_t *ibuf = (float32_t *)psd_fft_input;
	
	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;
	uint16_t overlap = psd_fft_overlap;	
	
	uint8_t *input = &adc_buffer[WISPR_DATA_HEADER_SIZE];
	float32_t *output = (float32_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE];
	
	// parse the adc buffer header to get the timestamp
	//wispr_data_header_t adc_hdr;
	//wispr_parse_data_header(adc_buffer, &adc_hdr);

	// update the data header timestamp
	//psd_data_header.second = adc_hdr.seconds; // epoch time stamp
	//psd_data_header.usec = adc_hdr.usec; // epoch time stamp
	rtc_get_epoch(&psd_data_header.second); // epoch time stamp	
	psd_data_header.usec = 0;

	// serialize the data buffer header
	wispr_serialize_data_header(&psd_data_header, psd_buffer);

	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - nfft)/(nfft - overlap));
	if(navg < 0) navg = 1;
	
	// scale is used to normalized the averaging
	float32_t scale = 1.0 / (float32_t)navg;
	
	uint16_t skip = nfft - overlap;
	
	//printf("Spectrum: %d %d %d %d %d\r\n", nfft, nbins, overlap, navg, skip);
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// clear output vector
	for(m = 0; m < nbins; m++) output[m] = 0;

	// average the time bins
	for(k = 0; k < navg; k++)  {
		
		iend = istart + nfft; // end of segment
		
		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < nfft; m++) ibuf[m] = 0.0;
		}
		
		// load the data window into the real fft buffer
		// units of ibuf are volts
		m = 0;
		for(n = istart; n < iend; n++) {
			// load the 24 or 16 bit word into a 32 bit float, preserving the sign bit
			if ( psd_sample_size == 3 ) {	
				uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
				ibuf[m] = win[m] * ((float32_t)((int32_t)uv >> 8)); //  // real
			} else if ( psd_sample_size == 2 ) {
				int16_t uv = (int16_t)( ((uint16_t)input[2*n+0] << 0) | ((uint16_t)input[2*n+1] << 8) );
				ibuf[m] = win[m] * (float32_t)uv;
			}
			//if((k == 0) && (n < istart+8)) printf("%.4f ", ibuf[m]);
			m++;
		}
		//if(k == 0) printf("\r\n\r\n");

		/* Process the data through the CFFT/CIFFT module */
		arm_rfft_fast_f32(&psd_twid_f32, ibuf, obuf, 0);
		
		// calc magnitude of the complex fft output stored in obuf
		//obuf[1] = 0;
		//arm_cmplx_mag_squared_q31(obuf, ibuf, nbins);
		
		// sum and save the magnitude of the complex fft output
		obuf[1] = 0; // dc component
		for(m = 0, n = 0; m < nbins; m++, n+=2) {
			//output[m] += scale * (obuf[n]*obuf[n] + obuf[n+1]*obuf[n+1]);
			output[m] += (obuf[n]*obuf[n] + obuf[n+1]*obuf[n+1]);
		}
		
		// increment the start index by skip=nsamps-overlap
		istart += skip;
		
	}
	
	// normalize the average
	for(n = 0; n < nbins; n++) output[n] = output[n] * scale;

	//printf(". finished\r\n");
		
	return(navg);
}


int spectrum_init_q31(wispr_config_t *wispr, uint16_t nbins, uint16_t nfft, uint16_t overlap, uint8_t bps, uint8_t wintype)
{
	// check spectrum size
	if( nbins > nfft/2 ) {
		nbins = nfft/2;
		//printf("spectrum_init_f32: number of bins greater than half fft size: %d\r\n", nbins);
	}

	// check number of bins
	if(nbins > PSD_MAX_BINS_PER_BUFFER) {
		nbins = PSD_MAX_BINS_PER_BUFFER;
		printf("spectrum_init_f32: number of bin truncated to %d\r\n", nbins);
	}
	psd_num_freq_bins = nbins;

	// check fft size
	if( nfft > PSD_MAX_FFT_SIZE ) {
		printf("spectrum_init_f32: unsupported fft size %d\r\n", nfft);
		return(ARM_MATH_LENGTH_ERROR);
	}
	psd_fft_size = nfft;
	
	// check overlap
	if( overlap >= psd_fft_size || overlap < 0  ) {
		printf("spectrum_init_f32: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_fft_overlap = overlap;
	
	// check bytes per sample
	if( (bps < 2) || (bps > 4)  ) {
		printf("spectrum_init_f32: unsupported sample size %d\r\n", bps);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_sample_size = bps;

	// pre-calculated twiddle factors
	arm_status status = arm_rfft_fast_init_f32(&psd_twid_f32, psd_fft_size);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window((float32_t *)psd_fft_window, wintype, psd_fft_size);
	
	//float32_t scale = 2147483648.0 / 8388608.0;
	float32_t scale = 2147483648.0;
	
	// convert window f32 values to q31
	q31_t *win_q31 = (q31_t *)psd_fft_window;
	float32_t *win_f32 = (float32_t *)psd_fft_window;
	for(int n = 0; n < psd_fft_size; n++) {
		win_q31[n] = (q31_t)(win_f32[n] * scale); // convert float to q31 by mult with 2^31
	}

	// update local data header structure with the current config
	// this is used to update the current data buffer header
	//wispr_update_data_header(wispr, &psd_data_header);
	
	psd_sampling_freq = wispr->sampling_rate; // samples per second (hz)
	psd_freq_bin_size = 1000 * psd_sampling_freq / psd_fft_size; // 1000 * hz/bin

	// update local data header structure with the current config
	wispr_update_data_header(wispr, &psd_data_header);
	psd_data_header.settings[0] = WISPR_SPECTRUM;

	// but change settings, sampling rate (1000*hz/bin), sample_size, ...
	psd_data_header.sampling_rate = psd_freq_bin_size;
	psd_data_header.sample_size = 4;
	psd_data_header.samples_per_block = psd_num_freq_bins;
	psd_data_header.block_size = PSD_MAX_BUFFER_SIZE; // number of bytes in an psd record block
	
	printf("spectrum_init_q31:  nfft=%d, nbins=%d, overlap=%d, bps=%d\r\n",
		psd_fft_size, psd_num_freq_bins, psd_fft_overlap, psd_sample_size);

	return(status);

}

//
// Calculate the spectrum for the input signal using a fixed point (q31) real fft.
// Data type a Q format numbers, so 
// 
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//
int spectrum_q31(uint8_t *psd_buffer, uint8_t *adc_buffer, uint16_t nsamps)
{
	int k, m, n;
	q31_t *win = (q31_t *)psd_fft_window;
	q31_t *buf1 = (q31_t *)psd_fft_output;
	q31_t *buf2 = (q31_t *)psd_fft_input;

	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;
	uint16_t overlap = psd_fft_overlap;

	// break out the hdr and data pointers
	uint8_t *input = &adc_buffer[WISPR_DATA_HEADER_SIZE];
	q31_t *output = (q31_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE];

	// parse the adc buffer header to get the timestamp
	//wispr_data_header_t adc_hdr;
	//wispr_parse_data_header(adc_buffer, &adc_hdr);

	// update the data header timestamp
	//psd_data_header.second = adc_hdr.seconds; // epoch time stamp
	//psd_data_header.usec = adc_hdr.usec; // epoch time stamp
	rtc_get_epoch(&psd_data_header.second); // epoch time stamp
	psd_data_header.usec = 0;

	// serialize the data header buffer
	wispr_serialize_data_header(&psd_data_header, psd_buffer);
	
	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - nfft)/(nfft - overlap));
	if(navg < 0) navg = 1;

	// find the number of bits to shift (upscale) the fft output  
	// this is needed because the fft input is downscaled by 2 for every fft stage to avoid saturation
	uint8_t scale_bits = 1;
	while( (nbins >> scale_bits) > 1 ) scale_bits++;
	
	// normalization for the output averaging
	q31_t norm = (q31_t)(2147483648.0 / (float32_t)navg); // convert float to q31 - mult by 2^31
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	uint16_t skip = nfft - overlap; // 
	
	// initialize magnitude vector
	for(m = 0; m < nbins; m++) output[m] = 0;

	// average the time bins
	for(k = 0; k < navg; k++)  {

		iend = istart + nfft; // end of segment
		
		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < nfft; m++) buf1[m] = 0;
		}
		
		// load the data window into the real fft buffer
		m = 0;
		for(n = istart; n < iend; n++) {
			// load the 24 or 16 bit word into a 32 bit float, preserving the sign bit
			if ( psd_sample_size == 3 ) {
				uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
				buf1[m] = (q31_t)((int32_t)uv >> 8); //  // real
			} else if ( psd_sample_size == 2 ) {
				int16_t uv = (int16_t)( ((uint16_t)input[2*n+0] << 0) | ((uint16_t)input[2*n+1] << 8) );
				buf1[m] = (q31_t)uv;
			}
			//if((k == 0) && (n < istart+8)) printf("%.4f ", ibuf[m]);
			m++;
		}
		//if(k == 0) printf("\r\n\r\n");

		// apply the window to the input buffer
		arm_mult_q31(buf1, win, buf2, nfft);

		//if(k == 0) {
		//	for(n = 0; n < 8; n++) printf("%x ", ibuf[n]);
		//	printf("\r\n");			
		//}
		
		/* Process the data through the CFFT/CIFFT module */
		arm_rfft_q31(&psd_twid_q31, buf2, buf1);

		// upshift the fft output inplace
		arm_shift_q31(buf1, scale_bits, buf1, nfft);

		// calc magnitude of the complex fft output stored in obuf
		buf1[1] = 0; // dc component
		arm_cmplx_mag_squared_q31(buf1, buf2, nbins);
		
		// accumulate the result in the output array to average
		//arm_add_q31(output, ibuf, output, nbins);
		for(m = 0; m < nbins; m++) {
			output[m] = buf2[m];
		}
		
		// increment the start index by skip=nsamps-overlap
		istart += skip;
		
	}

	// normalize the average
	//arm_mult_q31(buf1, win, buf2, nfft);
	//arm_scale_q31(output, norm, 0, output, nbins); // normalize the averaged output
	for(m = 0; m < nbins; m++) {
		//output[m] *= norm;
	}
	
	//printf("spectrum_q31:  %d %d, %d, %d\r\n", navg, nbins, nfft, overlap);
	return(navg);
}

