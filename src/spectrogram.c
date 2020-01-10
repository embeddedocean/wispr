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

#include "spectrogram.h"

// statically allocated data arrays so avoid runtime alloc
COMPILER_WORD_ALIGNED uint8_t psd_fft_input[4*MAX_FFT_SIZE];
COMPILER_WORD_ALIGNED uint8_t psd_fft_output[4*MAX_FFT_SIZE];

// FFT window
float32_t psd_window_f32[MAX_FFT_SIZE];
q31_t psd_window_q31[MAX_FFT_SIZE];

arm_rfft_instance_q31 psd_twid_q31;	// pre-calculated twiddle factors
arm_rfft_fast_instance_f32 psd_twid_f32;	// pre-calculated twiddle factors

uint16_t psd_fft_size;
uint16_t psd_num_freq_bins;  // nfft/2 +1

// 
// window function converted from matlab
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

int spectrum_init_f32(uint16_t size, uint8_t wintype)
{
	// check spectrum size
	if( (size != 32) && (size != 64) && (size != 128) && (size != 256) && (size != 512) && (size != 1024)) {
		printf("spectrum_init_f32: error, unsupported size %d", size);
		return(ARM_MATH_LENGTH_ERROR);
	}
	
	// check fft size 
	psd_fft_size = 2*size;
	if( psd_fft_size > MAX_FFT_SIZE ) {
		printf("spectrum_init: error, unsupported fft size %d", psd_fft_size);
		return(ARM_MATH_LENGTH_ERROR);
	}
	
	psd_num_freq_bins = size;
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_fast_init_f32(&psd_twid_f32, psd_fft_size);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_fast_init_f32 %d", status);
		return(status);
	}
	
	// generate window function
	spectrum_window(psd_window_f32, wintype, psd_fft_size);
	
	return(status);
}

//
// Calculate the spectrum for the input signal
// using a floating point (float32) real fft.
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//
int spectrum_f32(uint8_t *input, float32_t *output, uint16_t nsamps, uint16_t overlap)
{
	int k, m, n;
	float32_t *win = (float32_t *)psd_window_f32;
	float32_t *obuf = (float32_t *)psd_fft_output;
	float32_t *ibuf = (float32_t *)psd_fft_input;
	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;

	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - nfft)/(nfft - overlap));
	if(navg < 0) navg = 1;

	// scale is used to normalized the averaging
	float32_t scale = 1.0 / (float32_t)navg;
	
	uint16_t skip = nfft - overlap;
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// initialize magnitude vector
	for(m = 0; m < nbins; m++) output[m] = 0;

	// average the time bins
	for(k = 0; k < navg; k++)  {

		iend = istart + nfft; // end of segment

		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < 2*nfft; m++) ibuf[m] = 0.0;
		}

		// load the data window into the real part of complex fft buffer
		for(n = istart, m = 0; n < iend; n++, m+=2) {
			// load the 24 bit word into a 32 bit word, preserving the sign bit
			uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
			ibuf[m] = win[m] * (float32_t)((int32_t)uv >> 8); // real
		}

		/* Process the data through the CFFT/CIFFT module */
		arm_rfft_fast_f32(&psd_twid_f32, ibuf, obuf, 0);

		// sum and save the magnitude of the complex fft output
		arm_cmplx_mag_squared_f32(output, obuf, nbins);
		output[0] = obuf[0]; // dc component
		
		//for(m = 1, n = 2; m < nbins; m++, n+=2) {
		//	output[m] += obuf[n]*obuf[n] + obuf[n+1]*obuf[n+1];
		//}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// normalize the average
	for(n = 0; n < nbins; n++) output[n] = output[n] * scale;

	return(navg);
}

int spectrum_init_q31(uint16_t size, uint8_t wintype)
{
	// check spectrum size
	if( (size != 32) && (size != 64) && (size != 128) && (size != 256) && (size != 512) && (size != 1024)) {
		printf("spectrum_init: error, unsupported size %d", size);
		return(ARM_MATH_LENGTH_ERROR);
	}
	
	// check fft size
	psd_fft_size = 2*size;
	if( psd_fft_size > MAX_FFT_SIZE ) {
		printf("spectrum_init: error, unsupported fft size %d", psd_fft_size);
		return(ARM_MATH_LENGTH_ERROR);
	}
	
	psd_num_freq_bins = size;
	
	// pre-calculated twiddle factors
	arm_status status = arm_rfft_init_q31(&psd_twid_q31, psd_fft_size, 0, 0);
	if( status != ARM_MATH_SUCCESS) {
		printf("spectrum_init: error in arm_rfft_init_q31 %d", status);
		return(status);
	}
	
	// generate q31 window 
	float32_t temp[MAX_FFT_SIZE];
	spectrum_window(temp, wintype, psd_fft_size);
	for(int n = 0; n < psd_fft_size; n++) {
		psd_window_q31[n] = (q31_t)(temp[n] * 2147483648.0); // convert float to q31 - mult by 2^31
	}
	
	return(status);
}

//
// Calculate the spectrum for the input signal using a fixed point (q31) real fft.
// Data type a Q format numbers, so 
// 
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__RealFFT.html
//
int spectrum_q31(uint8_t *input, q31_t *output, uint16_t nsamps, uint16_t overlap)
{
	int k, m, n;
	q31_t *win = psd_window_q31;
	q31_t *obuf = (q31_t *)psd_fft_output;
	q31_t *ibuf = (q31_t *)psd_fft_input;
	uint16_t nfft = psd_fft_size;
	uint16_t nbins = psd_num_freq_bins;

	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - nfft)/(nfft - overlap));
	if(navg < 0) navg = 1;

	// find the number of bits to shift (upscale) the fft output  
	// this is needed because the fft input is downscaled by 2 for every fft stage to avoid saturation
	uint8_t scale_bits = 1;
	while( (nbins >> scale_bits) > 1 ) scale_bits++;
	
	// normalization for the output averaging
	float32_t fnorm = 1.0 / (float32_t)navg;
	q31_t norm = (q31_t)(fnorm * 2147483648.0); // convert float to q31 - mult by 2^31
	
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
			for(m = 0; m < 2*nfft; m++) ibuf[m] = 0.0;
		}

		// load the data window into the real part of complex fft buffer
		for(n = istart, m = 0; n < iend; n++, m+=2) {
			// load the 24 bit word into a 32 bit word, preserving the sign bit
			//uint32_t uv = (uint32_t)(input[3*n+0] << 8) | (input[3*n+1] << 16) | (input[3*n+2] << 24);
			uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
			ibuf[m] = win[n] * (q31_t)((int32_t)uv >> 8); 
		}
		
		// apply the window to the input buffer
		//arm_mult_q31(ibuf, win, ibuf, nfft);

		/* Process the data through the CFFT/CIFFT module */
		arm_rfft_q31(&psd_twid_q31, ibuf, obuf);
		arm_shift_q31(obuf, scale_bits, obuf, nbins); // upshift the fft output inplace

		// calc magnitude of the complex fft output stored in obuf
		//  using ibuf as the destination 
		arm_cmplx_mag_squared_q31(obuf, ibuf, nbins);
		ibuf[0] = obuf[0]; // dc component

		// sum the result in the output array to average
		arm_add_q31(output, ibuf, output, nbins);
		//for(m = 0; m < nbins; m++) {
		//	output[m] += ibuf[m];
		//}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// normalize the average
	arm_scale_q31(output, norm, 0, output, nbins); // normalize the averaged output

	return(navg);
}


/*
//
// Calculate the spectrum for the input signal
// using a floating point (float32) complex fft.
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__ComplexFFT.html
//
int spectrum_cfft_f32(uint8_t *input, float32_t *output, uint16_t nsamps, uint16_t overlap)
{
	int k, m, n;
	uint16_t fft_size = PSD_FFT_SIZE;
	float32_t mag[PSD_FFT_SIZE]; // real
	float32_t buf[2*PSD_FFT_SIZE]; // complex

	// pre-calculated twiddle factors
	const arm_cfft_instance_f32 *twid;
#if PSD_FFT_SIZE == 64
	twid = &arm_cfft_sR_f32_len64;
#elif PSD_FFT_SIZE == 128
	twid = &arm_cfft_sR_f32_len128;
#elif PSD_FFT_SIZE == 256
	twid = &arm_cfft_sR_f32_len256;
#elif PSD_FFT_SIZE == 512
	twid = &arm_cfft_sR_f32_len512;
#elif PSD_FFT_SIZE == 1024
	twid = &arm_cfft_sR_f32_len1024;
#elif PSD_FFT_SIZE == 2048
	twid = &arm_cfft_sR_f32_len2048;
#elif PSD_FFT_SIZE == 4096
	twid = &arm_cfft_sR_f32_len4096;
#endif

	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - fft_size)/(fft_size - overlap));
	if(navg < 0) navg = 1;

	float32_t norm = 1.0 / (float32_t)fft_size;	
	uint16_t skip = fft_size - overlap;
	
	// build the spectrogram using overlapping ffts 
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// initialize magnitude vector
	for(m = 0; m < fft_size; m++) mag[m] = 0;

	// average the time bins
	for(k = 0; k < navg; k++)  {

		iend = istart + fft_size; // end of segment

		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end 
			for(m = 0; m < 2*fft_size; m++) buf[m] = 0.0;
		}

		// load the data window into the real part of complex fft buffer
		for(n = istart, m = 0; n < iend; n++, m+=2) {
			// load the 24 bit word into a 32 bit word, preserving the sign bit
			uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
			buf[m] = (float32_t)((int32_t)uv >> 8); // real
			buf[m+1] = 0.0; // imag
		}

		// Process the data through the CFFT/CIFFT module 
		arm_cfft_f32(twid, buf, 0, 0);

		// Calculating the magnitude at each bin 
		arm_cmplx_mag_f32(buf, mag, fft_size);
			
		// sum and save the magnitude of the complex fft output
		// could take cabf after the sum
		for(n = 0; n < fft_size; n++) {
			output[n] += mag[n];
		}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// normalize the average
	for(n = 0; n < fft_size; n++) output[n] = output[n] * norm;

	// initialize psd structure for return
	return(navg);
}

//
// calculate the spectrum for the input signal
// using a fixed point (q31 or int32) complex fft
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__ComplexFFT.html
//
int spectrum_cfft_q31(uint8_t *input, int32_t *output, uint16_t nsamps, uint16_t overlap)
{
	int k, m, n;
	uint16_t nfft = psd_fft_size;
	int32_t mag = (q31_t *)psd_fft_output; // real
	int32_t buf = (q31_t *)psd_fft_input; // complex fft buffer

	// pre-calculated twiddle factors
	const arm_cfft_instance_q31 *twid;
#if PSD_FFT_SIZE == 64
	twid = &arm_cfft_sR_q31_len64;
#elif PSD_FFT_SIZE == 128
	twid = &arm_cfft_sR_q31_len128;
#elif PSD_FFT_SIZE == 256
	twid = &arm_cfft_sR_q31_len256;
#elif PSD_FFT_SIZE == 512
	twid = &arm_cfft_sR_q31_len512;
#elif PSD_FFT_SIZE == 1024
	twid = &arm_cfft_sR_q31_len1024;
#elif PSD_FFT_SIZE == 2048
	twid = &arm_cfft_sR_q31_len2048;
#elif PSD_FFT_SIZE == 4096
	twid = &arm_cfft_sR_q31_len4096;
#endif

	// number of spectral estimates in the psd
	uint16_t navg = ((nsamps - nfft)/(nfft - overlap));
	if(navg < 0) navg = 1;

	float32_t norm = 1.0 / (float32_t)nfft;
	uint16_t skip = nfft - overlap;
	
	// build the spectrogram using overlapping ffts
	int istart = 0;  // start index of segment
	int iend;  // end index of segment
	
	// initialize magnitude vector
	for(m = 0; m < nfft; m++) mag[m] = 0;

	// average the time bins
	for(k = 0; k < navg; k++)  {

		iend = istart + nfft; // end of segment

		// handle buffer at the end of the input signal
		if(iend > nsamps) {
			iend = nsamps;
			// fill buffer with zeros so it is assured to be padded at the end
			for(m = 0; m < 2*nfft; m++) buf[m] = 0.0;
		}

		// load the data window into the real part of complex fft buffer
		for(n = istart, m = 0; n < iend; n++, m+=2) {
			// load the 24 bit word into a 32 bit word, preserving the sign bit
			uint32_t uv = ((uint32_t)input[3*n+0] << 8) | ((uint32_t)input[3*n+1] << 16) | ((uint32_t)input[3*n+2] << 24);
			buf[m] = ((int32_t)uv >> 8); // real
			buf[m+1] = 0; // imag
		}

		// Process the data through the CFFT/CIFFT module
		arm_cfft_q31(twid, buf, 0, 0);

		// Calculating the magnitude at each bin
		arm_cmplx_mag_q31(buf, mag, nfft);
		
		// sum and save the magnitude of the complex fft output
		// could take cabf after the sum
		for(n = 0; n < nfft; n++) {
			output[n] += mag[n];
		}

		// increment the start index by skip=nsamps-overlap
		istart += skip;

	}

	// normalize the average
	for(n = 0; n < nfft; n++) output[n] = output[n] * norm;

	// initialize psd structure for return
	return(navg);
}

*/
