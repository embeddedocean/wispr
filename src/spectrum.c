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
COMPILER_WORD_ALIGNED float32_t fft_buffer1[2*PSD_MAX_FFT_SIZE+1];
COMPILER_WORD_ALIGNED float32_t fft_buffer2[2*PSD_MAX_FFT_SIZE+1]; 

// FFT window
COMPILER_WORD_ALIGNED float32_t fft_window[PSD_MAX_FFT_SIZE+1];

arm_rfft_fast_instance_f32 fft_twid_f32; // pre-calculated twiddle factors

arm_rfft_instance_q31 fft_twid_q31;	// pre-calculated twiddle factors

//const static arm_cfft_instance_f32 *psd_cfft_twid_f32; // pre-calculated twiddle factors

uint16_t fft_size;
uint16_t fft_overlap;
uint16_t num_freq_bins;  // nfft/2
float32_t fft_window_power;
float32_t fft_window_scaling;
uint8_t fft_window_type;
//float32_t fft_scaling;

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
	psd->samples_per_block = num_freq_bins;
	psd->block_size = PSD_MAX_BUFFER_SIZE; // number of bytes in an psd record block
	psd->sampling_rate = adc->sampling_rate;
	psd->second = 0; // time gets updated with the adc data time when the spectrum is processed
	psd->usec = 0;

	// copy the adc settings into the psd header
	// and set the psd specific settings
	psd->settings[PSD_SETTINGS_INDEX_SAMPLE_SIZE] = adc->sample_size; // 
	psd->settings[PSD_SETTINGS_INDEX_FFT_WINTYPE] = fft_window_type; // window type flag
	psd->settings[PSD_SETTINGS_INDEX_FFT_SIZE] = fft_size >> 4; // shifted 4 to fit into an 8 bit value
	psd->settings[PSD_SETTINGS_INDEX_FFT_OVERLAP] = fft_overlap >> 4; // shifted 4 to fit into an 8 bit value

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
	
	// Hamming window
	a0 = 0.54; a1 = 0.46; a2 = 0; a3 = 0; a4 = 0;
	
	switch (type) 
	{
		case HANN_WINDOW: // Hann window
			a0 = 0.5; a1 = 0.5; a2 = 0; a3 = 0; a4 = 0;
			break;
		case HAMMING_WINDOW: // Hamming window
			a0 = 0.54; a1 = 0.46; a2 = 0; a3 = 0; a4 = 0;
			break;
		case BLACKMAN_WINDOW:  // Blackman window 
			a0 = 0.42; a1 = 0.5; a2 = 0.08; a3 = 0; a4 = 0;
			break;
		case RECT_WINDOW: // rectangular window
		default:
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

int spectrum_init_f32(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint8_t wintype)
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
	num_freq_bins = *nbins;
	
	// check fft size
	if( nfft > PSD_MAX_FFT_SIZE ) {
		printf("spectrum_init_f32: unsupported fft size %d\r\n", nfft);
		return(ARM_MATH_LENGTH_ERROR);
	}
	fft_size = nfft;
	
	// check overlap
	if( (overlap >= fft_size) || (overlap < 0)  ) {
		printf("spectrum_init_f32: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	fft_overlap = overlap;
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_fast_init_f32(&fft_twid_f32, fft_size);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window(fft_window, wintype, fft_size);
	
	// calc the window power to use when scaling the fft output 
	arm_power_f32(fft_window, fft_size, &fft_window_power);
	
	// window scaling factor
	//fft_window_scaling = ADC_SCALING / max_value;
	fft_window_scaling = 1.0f;
	if(wintype == HAMMING_WINDOW) fft_window_scaling = 2.0f;
	else if(wintype == HANN_WINDOW) fft_window_scaling = 2.0f;
	
	// apply the scaling factor to the window
	for(n = 0; n < fft_size; n++) {
		fft_window[n] *= fft_window_scaling;
	}
	//for(int n = 0; n < fft_size; n++) printf("%.2f ", fft_window_f32[n]);
	//printf("\r\n");
	
	//printf("spectrum_init_f32:  nfft=%d, nbins=%d, overlap=%d, bps=%d, fft_window_power=%f\r\n",
	//	fft_size, psd_num_freq_bins, fft_overlap, nbps, fft_window_power);
	printf("spectrum_init_f32: fft_window_power=%f\r\n", fft_window_power);
	
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
	
	float32_t *win = fft_window;
	float32_t *buf2 = fft_buffer2;
	float32_t *buf1 = fft_buffer1;
	
	uint16_t nfft = fft_size;
	uint16_t nbins = num_freq_bins;
	uint16_t overlap = fft_overlap;	
	
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
	
	// fft input scaling to prevent overflow or underflow
	//float32_t adc_scaling;
	uint8_t fft_shift_bits = 0;
	if( sample_size == 3) { // 24 bits per sample
		//adc_scaling = ADC_SCALING / 8388607.0; // 2^23 - 1
		fft_shift_bits = 0;
	}
	else if( sample_size == 2 ) { // 16 bits per sample
		//adc_scaling = ADC_SCALING / 32767.0; // 2^15 - 1
		fft_shift_bits = 8;
	}
		
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
			//printf("zero padding\r\n");
		}
		
		// Load the data window into the real fft buffer.
		// The adc data word is left shifted to fill the msb of the fft buffer.
		// This is done to prevent the fft output buffer values from getting to small.
		// This amounts to a scaling that will need to be removed later.
		// Bit shifting behavior is undefined for signed numbers and
		// if the number is shifted more than the size of integer, so cast before shifting.
		m = 0;
		if( sample_size == 2 ) {
			// load the 16 bit word into a 32 bit float
			for(n = istart; n < iend; n++, m++) {
				//int32_t v = (int32_t)(((uint32_t)input[2*n+0] << 16) | ((uint32_t)input[2*n+1] << 24));
				//buf1[m] = (float32_t)v;
				buf1[m] = (float32_t)(LOAD_INT16(input,n) << fft_shift_bits);
			}
		} else if ( sample_size == 3 ) {
			// load the 24 bit word into a 32 bit float
			for(n = istart; n < iend; n++, m++) {
				//int32_t v = (int32_t)(((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24));
				//buf1[m] = (float32_t)v;
				buf1[m] = (float32_t)(LOAD_INT24(input,n) << fft_shift_bits);
			}
		} else {
			printf("spectrum_f32: unsupported sample size\r\n");
			return(0);
		}

		// apply the window to the input buffer
		// could skip if rect window is used
		arm_mult_f32(buf1, win, buf2, (uint32_t)nfft);
		
		//if(k == 0 ) {
		//	printf("ibuf = [");
		//	for(n = 0; n < nfft; n++) {
		//		printf("%f ", buf2[n]);
		//	}
		//	printf("];\r\n");	
		//}
		
		/* Process the data using rfft */
		arm_rfft_fast_f32(&fft_twid_f32, buf2, buf1, 0);

		//if(k == 0 ) {
		//	printf("obuf = [\r\n");
		//	for(n = 0; n < nbins; n++) {
		//		printf("%d+i*%d ", buf1[2*n], buf1[2*n+1]);
		//	}
		//	printf("];\r\n");
		//}
		
		// scale the output of the fft so the value don't get too small
		//arm_scale_f32(buf1, fft_scaling, buf2, (uint32_t)nfft);

		// calc magnitude of the complex fft output
		buf1[1] = 0; // dc
		arm_cmplx_mag_squared_f32(buf1, buf2, (uint32_t)nbins);
		
		// accumulate for mean
		//arm_add_f32(buf2, output, output, (uint32_t)nbins);
		for(m = 0; m < nbins; m++) {
			output[m] += buf2[m];
		}
		
		// increment the start index by skip=nsamps-overlap
		istart += skip;
		
	}
	
	// normalization and remove the fft scaling 
	// Note that no adc scaling is applied because the numbers get too small.
	// So apply the adc scaling later.
	float32_t fft_scaling = 1.0f / (float32_t)(1 >> fft_shift_bits);
	float32_t norm = 2.0f * (fft_scaling * fft_scaling) / (float32_t)navg;

	// remove window scaling
	norm *= 1.0f / (fft_window_scaling * fft_window_scaling);

	// remove window power??
	//norm *= 1.0f / fft_window_power;
	
	// Normalize the output 
	// Because the signal is real-valued, you only need power estimates for the positive frequencies. 
	// To conserve the total power, multiply all frequencies by a factor of 2. 
	// however, zero frequency (DC) and the Nyquist frequency do not occur twice.
	output[0] *= (norm / 2.0f);
	for(n = 1; n < nbins; n++) output[n] *= norm;

	return((int)navg);
}


//q31_t *fft_window_q31 = (q31_t *)fft_window;

int spectrum_init_q31(uint16_t *nbins, uint16_t nfft, uint16_t overlap, uint8_t wintype)
{	
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
	num_freq_bins = *nbins;
	
	// check fft size
	if( nfft > PSD_MAX_FFT_SIZE ) {
		printf("spectrum_init_q31: unsupported fft size %d\r\n", nfft);
		return(ARM_MATH_LENGTH_ERROR);
	}
	fft_size = nfft;
	
	// check overlap
	if( (overlap >= fft_size) || (overlap < 0)  ) {
		printf("spectrum_init_q31: unsupported fft overlap size %d\r\n", overlap);
		return(ARM_MATH_ARGUMENT_ERROR);
	}
	fft_overlap = overlap;
	
	//psd_sampling_freq = fs; // samples per second (hz)
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_init_q31(&fft_twid_q31, fft_size, 0, 1);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init_q31: error in arm_rfft_init_q31 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window(fft_window, wintype, fft_size);
	
	// calc the window power to use when scaling the fft output
	arm_power_f32(fft_window, fft_size, &fft_window_power);
	
	// window scaling factor
	fft_window_scaling = 1.0f;
	if(wintype == HAMMING_WINDOW) fft_window_scaling = 2.0f;
	else if(wintype == HANN_WINDOW) fft_window_scaling = 2.0f;
	
	// apply the scaling factor to the window
	for(int n = 0; n < fft_size; n++) {
		fft_window[n] *= fft_window_scaling;
	}

	//printf("spectrum_init_q31:  nfft=%d, overlap=%d, bps=%d, win_power=%f\r\n",
	//	fft_size, fft_overlap, nbps, fft_window_power);

	return(status);
}

//
// Calculate the spectrum for the input signal using a fixed point (q31) real fft.
// Data type is Q format numbers.
//
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//

int spectrum_q31(wispr_data_header_t *psd, float32_t *psd_data, wispr_data_header_t *adc, uint8_t *adc_data, uint16_t nsamps)
{
	uint16_t k, m, n;

	//q31_t *win = fft_window_q31;
	float32_t *win = fft_window;
	q31_t *buf1 = (q31_t *)fft_buffer1;
	q31_t *buf2 = (q31_t *)fft_buffer2;
	
	uint16_t nfft = fft_size;
	uint16_t nbins = num_freq_bins;
	uint16_t overlap = fft_overlap;
	
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

	// fft input scaling to prevent overflow or underflow
	//float32_t adc_scaling;
	uint8_t fft_shift_bits = 0;
	if( sample_size == 3) { // 24 bits per sample
		//adc_scaling = ADC_SCALING / 8388607.0; // 2^23 - 1
		fft_shift_bits = 0;
	}
	else if( sample_size == 2 ) { // 16 bits per sample
		//adc_scaling = ADC_SCALING / 32767.0; // 2^15 - 1
		fft_shift_bits = 8;
	}
		
	// clear output vector
	for(m = 0; m < nbins; m++) output[m] = 0.0;

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
		
		// Load the data window into the q31 fft buffer.
		// The adc data word is left shifted to fill the msb of the fft buffer.
		// This is done to prevent the fft output buffer values from getting to small
		// amounting to a scaling that will need to be removed later.
		// Bit shifting behavior is undefined for signed numbers and
		// if the number is shifted more than the size of integer, so cast before shifting.
		m = 0;
		if( sample_size == 2 ) {
			// load the 16 bit word into a 32 bit q31_t
			for(n = istart; n < iend; n++) {
				//int32_t v = (int32_t)( ((uint32_t)input[2*n+0] << 16) | ((uint32_t)input[2*n+1] << 24) );
				//buf1[m] = (q31_t)(win[m] * (float32_t)v);
				buf1[m] = (q31_t)(win[m] * (float32_t)(LOAD_INT16(input,n) << fft_shift_bits));
				m++;
			}
		} else if ( sample_size == 3 ) {
			// load the 24 bit word into a 32 bit q31
			for(n = istart; n < iend; n++) {
				//int32_t v = (int32_t)(((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24));
				//buf1[m] = (q31_t)(win[m] * (float32_t)v);
				buf1[m] = (q31_t)(win[m] * (float32_t)(LOAD_INT24(input,n) << fft_shift_bits));
				m++;
			}
		} else {
			printf("spectrum_q31: unsupported sample size\r\n");
			return(0);
		}

		//if(k == 0 ) {
		//	printf("ibuf = [\r\n");
		//	for(n = 0; n < nfft; n++) {
		//		printf("%d ", buf1[n]);
		//	}
		//	printf("];\r\n");
		//}

		// Process the data through the RFFT module 
		// rfft_q31 seems to require buffer sizes of 2*nfft
		arm_rfft_q31(&fft_twid_q31, buf1, buf2);
		
		// calc magnitude of the complex fft output and 
		// accumulate the result in the output array to average
		m=0;
		buf2[1] = 0;
		for(n = 0; n < nbins; n++) {
			float32_t re = (float32_t)buf2[m++]; 
			float32_t im = (float32_t)buf2[m++]; 
			output[n] += (re*re + im*im);
		}

		// this would be faster but it doesn't work, not sure why, maybe overflow
		// calc magnitude of the complex fft output
		//buf2[1] = 0; // dc component
		//arm_cmplx_mag_squared_q31(buf2, buf1, (uint32_t)nbins);
		// accumulate the result in the output array to average
		//for(n = 0; n < nbins; n++) {
		//	output[n] += (float32_t)buf1[n];
		//}
		
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

	// normalization and remove the fft scaling
	// It also needs to be scaled by nfft because the arm_rfft_q31 removes this
	// Note that no adc scaling is applied because the numbers get too small.
	// So apply the adc scaling later.
	float32_t fft_scaling = (float32_t)nfft / (float32_t)(1 >> fft_shift_bits);
	float32_t norm = 2.0f * (fft_scaling * fft_scaling) / (float32_t)navg;

	// remove window scaling
	norm *= 1.0f / (fft_window_scaling * fft_window_scaling);
		
	// Normalize the output
	// Because the signal is real-valued, you only need power estimates for the positive frequencies.
	// To conserve the total power, multiply all frequencies by a factor of 2.
	// however, zero frequency (DC) and the Nyquist frequency do not occur twice.
	output[0] = output[0] * norm / 2.0;
	for(n = 1; n < nbins; n++) output[n] *= norm;
	
	return((int)navg);
}


