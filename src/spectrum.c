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
 * Embedded Ocean Systems (EOS), 2020
 *
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <compiler.h>
#include <math.h>
#include <float.h>

#include "rtc_time.h"
#include "wispr.h"
#include "spectrum.h"

//
// statically allocated data arrays so avoid runtime allocation
//
// the documentation says that the fft buffers should be the same size as the fft
// but this fails for rfft_q31, so make them 2*nfft
COMPILER_WORD_ALIGNED float32_t psd_fft_buffer1[2*PSD_MAX_FFT_SIZE+1];
COMPILER_WORD_ALIGNED float32_t psd_fft_buffer2[2*PSD_MAX_FFT_SIZE+1]; 

// FFT window
COMPILER_WORD_ALIGNED float32_t psd_fft_window[PSD_MAX_FFT_SIZE+1];

arm_rfft_fast_instance_f32 psd_twid_f32; // pre-calculated twiddle factors

arm_rfft_instance_q31 psd_twid_q31;	// pre-calculated twiddle factors

//const static arm_cfft_instance_f32 *psd_cfft_twid_f32; // pre-calculated twiddle factors

uint16_t psd_fft_size;
uint16_t psd_fft_overlap;
uint16_t psd_num_freq_bins;  // nfft/2
uint32_t psd_sampling_freq; // samples per second
uint32_t psd_freq_bin_size; // hz/bin
float32_t psd_window_power;
float32_t psd_window_scaling;
float32_t psd_fft_scaling;

// local instance of the wispr data header to use for updating the data buffer
//static wispr_data_header_t psd_data_header;


void spectrum_update_header(wispr_data_header_t *psd, wispr_data_header_t *adc)
{
	// update the psd header with the adc data header
	// make sure to set all the fields because this is what gets written to storage
	psd->version[0] = adc->version[0];
	psd->version[1] = adc->version[1];
	psd->type = WISPR_SPECTRUM;
	psd->sample_size = 4;
	psd->samples_per_block = psd_num_freq_bins;
	psd->block_size = PSD_MAX_BUFFER_SIZE; // number of bytes in an psd record block
	psd->sampling_rate = adc->sampling_rate;
	psd->second = 0; // time gets updated with the adc data time when the spectrum is processed
	psd->usec = 0;

	// copy the adc settings into the psd header
	// and set the psd specific settings
	psd->settings[0] = adc->settings[0]; // this is adc gain, if the adc has been initialized;
	psd->settings[1] = adc->settings[1]; // adc df
	psd->settings[2] = psd_fft_size >> 4; // shifted 4 to fit into an 8 bit value
	psd->settings[3] = psd_fft_overlap >> 4; // shifted 4 to fit into an 8 bit value

	// update the data header timestamp
	psd->second = adc->second; // epoch time stamp
	psd->usec = adc->usec; // epoch time stamp

}

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
		case RECT_WINDOW:
			// rectangular window
			for(int m = 0; m < size; m++) w[m] = 1.0;
			return;
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

int spectrum_init_f32(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint32_t fs, uint8_t nbps, uint8_t wintype)
{
	uint16_t n = 0;
	
	// check spectrum size
	if( *nbins > nfft/2 ) {
		*nbins = nfft/2;
		printf("spectrum_init_f32: number of bin truncated to %d\r\n", *nbins);
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
	if( (overlap >= psd_fft_size) || (overlap < 0)  ) {
		printf("spectrum_init_f32: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_fft_overlap = overlap;
	
	psd_sampling_freq = fs; // samples per second (hz)
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_fast_init_f32(&psd_twid_f32, psd_fft_size);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	  
	// generate window function
	spectrum_window(psd_fft_window, wintype, psd_fft_size);
	
	// calc the window power to use when scaling the fft output 
	arm_power_f32(psd_fft_window, psd_fft_size, &psd_window_power);
	
	// max adc value used for scaling
	float32_t max_value  = 2147483647.0; // 2^31-1
	if( nbps == 3) { // 24 bits per sample
		max_value = 8388607.0; // 2^23 - 1
	}
	else if( nbps == 2 ) { // 16 bits per sample
		max_value = 32767.0; // 2^15 - 1
	}
	
	// apply the scaling factor to the window 
	psd_window_scaling = ADC_SCALING / max_value;
	for(n = 0; n < psd_fft_size; n++) {
		psd_fft_window[n] *= psd_window_scaling;
	}
	//for(int n = 0; n < psd_fft_size; n++) printf("%.2f ", psd_fft_window_f32[n]);
	//printf("\r\n");

	psd_fft_scaling = (float32_t)nfft;; // * ADC_SCALING / max_value; // 2^23

	//printf("spectrum_init_f32:  nfft=%d, nbins=%d, overlap=%d, bps=%d, win_power=%f\r\n",
	//	psd_fft_size, psd_num_freq_bins, psd_fft_overlap, nbps, psd_window_power);

	return(status);

}

//
// Calculate the nonparametric power spectral density (PSD) estimates equivalent to the periodogram
// using a floating point (float32) real fft.
// To obtain the psd in units of dB/hz, multiple the output by 1/samping_rate
//  dBV_per_hz = 10*log10(psd/Fs/Nfft) 
// For reference see:
//  https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//  https://www.mathworks.com/help/signal/ug/power-spectral-density-estimates-using-fft.html
// 
int spectrum_f32(wispr_data_header_t *psd, float32_t *psd_data, wispr_data_header_t *adc, uint8_t *adc_data, uint16_t nsamps)
{
	uint16_t k, m, n;
	
	float32_t *win = psd_fft_window;
	float32_t *buf2 = psd_fft_buffer2;
	float32_t *buf1 = psd_fft_buffer1;
	
	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;
	uint16_t overlap = psd_fft_overlap;	
	
	uint8_t *input = adc_data;
	float32_t *output = (float32_t *)psd_data;
	
	uint8_t sample_size = adc->sample_size;
	
	if(nsamps > adc->samples_per_block) {
		nsamps = adc->samples_per_block;
		printf("spectrum_f32: number of samples truncated to %d\r\n", nsamps);
	}	
	
	// update the data header
	spectrum_update_header(psd, adc);
	
	// number of spectral estimates in the averaged psd
	uint16_t skip = nfft - overlap;
	uint16_t navg = (nsamps - overlap) / skip;
	if(navg < 0) navg = 1;
	
	//printf("Spectrum: %d %d %d %d %d\r\n", nfft, nbins, overlap, navg, skip);
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// clear output vector
	for(m = 0; m < nbins; m++) output[m] = 0.0;
	
	// average the time bins
	for(k = 0; k < navg; k++)  {
		
		iend = istart + nfft; // end of segment
		
		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < nfft; m++) buf1[m] = 0.0;
			printf("zero padding\r\n");
		}
		
		// load the data window into the real fft buffer
		// units of the ibuf are volts because the window contains the adc scaling factor
		m = 0;
		if( sample_size == 2 ) {
			// load the 16 bit word into a 32 bit float
			for(n = istart; n < iend; n++, m++) {
				int16_t uv = (int16_t)( ((uint16_t)input[2*n+0] << 0) | ((uint16_t)input[2*n+1] << 8) );
				//buf1[m] = win[m] * (float32_t)uv;
				buf1[m] = (float32_t)uv;
			}
		} else if ( sample_size == 3 ) {
			// load the 24 bit word into a 32 bit float, preserving the sign bit
			for(n = istart; n < iend; n++, m++) {
				uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
				//buf1[m] = win[m] * ((float32_t)((int32_t)uv >> 8)); // this shift preserves the sign bit
				buf1[m] = ((float32_t)((int32_t)uv >> 8)); // this shift preserves the sign bit
			}
		} else {
			printf("spectrum_f32: unsupported sample size\r\n");
			return(0);
		}

		// apply the window to the input buffer
		arm_mult_f32(buf1, win, buf2, (uint32_t)nfft);
		
		//if(k == 0 ) {
		//	printf("ibuf(:,%d) = [", k+1);
		//	for(n = 0; n < nfft; n++) {
		//		printf("%.4f ", ibuf[n]);
		//	}
		//	printf("];\r\n");	
		//}
		
		/* Process the data using rfft */
		arm_rfft_fast_f32(&psd_twid_f32, buf2, buf1, 0);
		
		// calc magnitude of the complex fft output stored in obuf
		//buf1[1] = 0;
		//arm_cmplx_mag_squared_f32(buf1, buf2, (uint32_t)nbins);
		//for(m = 0; m < nbins; m++) {
		//	output[m] += buf2[m];
		//}
		// sum and save the magnitude of the complex fft output
		m=0;
		buf1[1] = 0;
		for(n = 0; n < nbins; n++) {
			float32_t re = buf1[m++] * psd_fft_scaling;
			float32_t im = buf1[m++] * psd_fft_scaling;
			// could check values against FLT_EPSILON for underflow
			output[n] += (re*re + im*im);
		}
		
		//if(k == 0 ) {
		//m = 0;
		//printf("buf(:,%d) = [", k+1);
		//for(m = 0; m < nbins; m++) {
		//	float32_t re = (float32_t)buf1[m++] * scale;
		//	float32_t im = (float32_t)buf1[m++] * scale;
		//	//printf("%f+i*%f ", re, im]);
		//	printf("%f ", (re*re + im*im));
		//}
		//printf("];\r\n");
		//}

		// increment the start index by skip=nsamps-overlap
		istart += skip;
		
	}
		
	// scale is used to normalizing
	float32_t norm = 2.0 / (float32_t)navg; 
	norm *= 1.0 / (psd_fft_scaling * psd_fft_scaling);
		
	// Normalize the output 
	// Because the signal is real-valued, you only need power estimates for the positive frequencies. 
	// To conserve the total power, multiply all frequencies by a factor of 2. 
	// however, zero frequency (DC) and the Nyquist frequency do not occur twice.
	output[0] = output[0] * norm / 2.0;
	for(n = 1; n < nbins; n++) output[n] *= norm;

	return((int)navg);
}



int spectrum_init_q31(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint32_t fs, uint8_t nbps, uint8_t wintype)
{
	uint16_t n = 0;
	
	// check spectrum size
	if( *nbins > nfft/2 ) {
		*nbins = nfft/2;
		printf("spectrum_init_q31: number of bin truncated to %d\r\n", *nbins);
	}
	
	// check number of bins
	if(*nbins > PSD_MAX_BINS_PER_BUFFER) {
		*nbins = PSD_MAX_BINS_PER_BUFFER;
		printf("spectrum_init_q31: number of bin truncated to %d\r\n", *nbins);
	}
	psd_num_freq_bins = *nbins;
	
	// check fft size
	if( nfft > PSD_MAX_FFT_SIZE ) {
		printf("spectrum_init_q31: unsupported fft size %d\r\n", nfft);
		return(ARM_MATH_LENGTH_ERROR);
	}
	psd_fft_size = nfft;
	
	// check overlap
	if( (overlap >= psd_fft_size) || (overlap < 0)  ) {
		printf("spectrum_init_q31: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	psd_fft_overlap = overlap;
	
	psd_sampling_freq = fs; // samples per second (hz)
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_init_q31(&psd_twid_q31, psd_fft_size, 0, 1);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init_q31: error in arm_rfft_init_q31 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window(psd_fft_window, wintype, psd_fft_size);
	
	psd_window_scaling = 1.0;

	// calc the window power to use when scaling the fft output
	arm_power_f32(psd_fft_window, psd_fft_size, &psd_window_power);

	// max adc value used for scaling
	float32_t max_value  = 2147483647.0; // 2^31-1
	if( nbps == 3) { // 24 bits per sample
		max_value = 8388607.0; // 2^23 - 1
	}
	else if( nbps == 2 ) { // 16 bits per sample
		max_value = 32767.0; // 2^15 - 1
	}
	
	psd_fft_scaling = (float32_t)nfft * ADC_SCALING / max_value; // 2^23
	
	//printf("spectrum_init_q31:  nfft=%d, nbins=%d, overlap=%d, bps=%d, win_power=%f\r\n",
	//	psd_fft_size, psd_num_freq_bins, psd_fft_overlap, nbps, psd_window_power);

	return(status);
}

//
// Calculate the spectrum for the input signal using a fixed point (q31) real fft.
// Data type a Q format numbers, so
//
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//

int spectrum_q31(wispr_data_header_t *psd, float32_t *psd_data, wispr_data_header_t *adc, uint8_t *adc_data, uint16_t nsamps)
{
	uint16_t k, m, n;

	float32_t *win = psd_fft_window;
	q31_t *buf1 = (q31_t *)psd_fft_buffer1;
	q31_t *buf2 = (q31_t *)psd_fft_buffer2;
	
	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;
	uint16_t overlap = psd_fft_overlap;
	
	uint8_t *input = adc_data;
	float32_t *output = psd_data;
	
	// update the data header
	spectrum_update_header(psd, adc);
	uint8_t sample_size = adc->sample_size;
	
	// number of spectral estimates in the psd
	uint16_t skip = nfft - overlap;
	uint16_t navg = (nsamps - overlap) / skip;
	if(navg < 0) navg = 1;
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// clear output vector
	for(m = 0; m < nbins; m++) output[m] = 0.0;

	// find the number of bits to shift (upscale) the fft output
	// this is needed because the fft input is downscaled by 2 for every fft stage to avoid saturation
	//uint8_t scale_bits = 1;
	//while( (nbins >> scale_bits) > 1 ) scale_bits++;

	//printf("Spectrum_q31: %d %d %d %d %d %d\r\n", nsamps, nfft, nbins, overlap, navg, skip);
	
	// average the time bins
	for(k = 0; k < navg; k++)  {
		
		iend = istart + nfft; // end of segment
		
		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < nfft; m++) buf1[m] = 0;
		}
		
		// load the data window into the fft buffer
		m = 0;
		if( sample_size == 2 ) {
			// load the 16 bit word into a 32 bit float
			for(n = istart; n < iend; n++) {
				int16_t uv = (int16_t)( ((uint16_t)input[2*n+0] << 0) | ((uint16_t)input[2*n+1] << 8) );
				//buf1[m] = (q31_t)uv;
				buf1[m] = (q31_t)(win[m] * (float32_t)uv);
				m++;
			}
		} else if ( sample_size == 3 ) {
			// load the 24 bit word into a 32 bit float, preserving the sign bit
			for(n = istart; n < iend; n++) {
				uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
				//buf1[m] = ((q31_t)((int32_t)uv >> 8)); // this shift preserves the sign bit
				buf1[m] = (q31_t)( win[m] * (float32_t)((int32_t)uv >> 8) );
				m++;
			}
		} else {
			printf("spectrum_q31: unsupported sample size\r\n");
			return(0);
		}

		// Process the data through the RFFT module 
		// rfft_q31 seems to require buffer sizes of 2*nfft
		arm_rfft_q31(&psd_twid_q31, buf1, buf2);
		
		// accumulate the result in the output array to average
		m=0;
		buf2[1] = 0;
		for(n = 0; n < nbins; n++) {
			float32_t re = (float32_t)buf2[m++] * psd_fft_scaling;
			float32_t im = (float32_t)buf2[m++] * psd_fft_scaling;
			output[n] += (re*re + im*im);
		}
		
		//if(k == 0 ) {
		//	printf("obuf = [\r\n");
		//	for(n = 0; n < nbins; n++) {
		//		printf("%d+i*%d ", buf2[2*n], buf2[2*n+1]);
		//	}
		//	printf("];\r\n");
		//}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// scale is used to normalizing
	float32_t norm = 2.0  / ((float32_t)navg);
	
	// Normalize the output
	// Because the signal is real-valued, you only need power estimates for the positive frequencies.
	// To conserve the total power, multiply all frequencies by a factor of 2.
	// however, zero frequency (DC) and the Nyquist frequency do not occur twice.
	output[0] = output[0] * norm / 2.0;
	for(n = 1; n < nbins; n++) output[n] *= norm;

	return((int)navg);
}

/*

int init_spectrum_q31(uint16_t nfft, uint8_t win_type )
{
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_init_q31(&psd_twid_q31, nfft, 0, 1);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	
	// generate window function
	window(win_f32, win_type, nfft);
	
	// calc the window power to use when scaling the fft output
	arm_power_f32(win_f32, nfft, &win_power);
	
	//float32_t scale = 2147483648.0; //2^31
	float32_t scale = 8388608.0; // 2^23
	
	// The algorithm used with fixed-point data is:
	//		pDst[n] = (pSrc[n] * scaleFract) << shift,   0 <= n < blockSize.
	// The overall scale factor applied to the fixed-point data is
	//		scale = scaleFract * 2^shift.
	// arm_scale_q31 (const q31_t *pSrc, q31_t scaleFract, int8_t shift, q31_t *pDst, uint32_t blockSize);
	
	// convert window f32 values to q31
	//for(int n = 0; n < nfft; n++) {
	//	win_q31[n] = (q31_t)(win_f32[n] * scale);
	//}
	
	return(status);
}


int spectrum_q31(q31_t *input, float32_t *output, uint16_t nsamps, uint16_t nbins, uint16_t nfft, uint16_t overlap )
{
	uint16_t k, m, n;

	q31_t *buf1 = (q31_t *)spec_buf1;
	q31_t *buf2 = (q31_t *)spec_buf2;
	
	// number of spectral estimates in the psd
	uint16_t skip = nfft - overlap;
	uint16_t navg = (nsamps - overlap) / skip;
	if(navg < 0) navg = 1;
	
	//printf("Spectrum: %d %d %d %d %d\r\n", nfft, nbins, overlap, navg, skip);
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// clear output vector
	for(m = 0; m < nbins; m++) output[m] = 0.0;

	// find the number of bits to shift (upscale) the fft output
	// this is needed because the fft input is downscaled by 2 for every fft stage to avoid saturation
	uint8_t scale_bits = 1;
	while( (nbins >> scale_bits) > 1 ) scale_bits++;

	float32_t scale = (float32_t)nfft * 5.0 / 8388608.0; // 2^23
	
	// average the time bins
	for(k = 0; k < navg; k++)  {
		
		iend = istart + nfft; // end of segment
		
		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < nfft; m++) buf1[m] = 0.0;
		}
		
		// load the data window into the real fft buffer
		// units of the ibuf are volts because the window contains the adc scaling factor
		m = 0;
		for(n = istart; n < iend; n++, m++) {
			buf1[m] = (q31_t)(win_f32[m] * (float32_t)input[n]);
			//buf1[m] = input[n];
		}

		//arm_scale_q31 (const q31_t *pSrc, q31_t scaleFract, int8_t shift, q31_t *pDst, uint32_t blockSize);
		
		// apply the window to the input buffer
		//arm_mult_q31(buf1, win_q31, buf2, nfft);

		//if(k == 0 ) {
		//	printf("ibuf = [\r\n");
		//	for(n = 0; n < nfft; n++) {
		//		printf("%d ", buf2[n]);
		//	}
		//	printf("];\r\n");
		//}
		
		// Process the data through the RFFT module
		arm_rfft_q31(&psd_twid_q31, buf1, buf2);
		
		//if(k == 0 ) {
		//	printf("buf = [\r\n");
		//	for(n = 0; n < nbins; n++) {
		//		printf("%d+i*%d ", buf2[2*n], buf2[2*n+1]);
		//	}
		//	printf("];\r\n");
		//}

		// upshift the fft output
		//		arm_shift_q31(buf1, scale_bits, buf2, nfft);

		// calc magnitude of the complex fft output stored in obuf
		//buf2[1] = 0; // dc component
		//arm_cmplx_mag_squared_q31(buf2, buf1, nbins);

		// accumulate the result in the output array to average
		//arm_add_q31(output, buf2, output, nbins);
		m=0;
		for(n = 0; n < nbins; n++) {
			//output[n] += buf1[n];
			float32_t re = (float32_t)buf2[m++] * scale;
			float32_t im = (float32_t)buf2[m++] * scale;
			output[n] += (re*re + im*im);
		}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// scale is used to normalizing
	scale = 2.0  / ((float32_t)navg);
	//q31_t norm = (q31_t)(scale * 2147483648.0);
	
	// Normalize the output
	// Because the signal is real-valued, you only need power estimates for the positive frequencies.
	// To conserve the total power, multiply all frequencies by a factor of 2.
	// however, zero frequency (DC) and the Nyquist frequency do not occur twice.
	output[0] *= scale/2;
	for(n = 1; n < nbins; n++) output[n] *= scale;

	return((int)navg);
}

*/