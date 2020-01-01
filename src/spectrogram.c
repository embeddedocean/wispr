/*
 * WISPR spectrogram function
 *
 * USAGE:
 * int spectrogram_init(spectrogram_t *psd, int fft_size, int overlap, int fs)
 * int spectrogram(spectrogram_t *psd, u_int32_t *input, int nsamps, int navg, int scaling_method, int shift)
 * 
 * DESCRIPTION:
 *
 * Spectrogram using a Short-Time Fourier Transform (STFT) to calculate the spectrogram 
 * of the signal specified by the vector 'input'. 
 * 
 * The spectrogram is formed by dividing the input into overlapping segments of length equal, 
 * multiplying the segment by a Hamming window of equal length, performing an FFT, and then
 * averaging the magnitude of the FFT over a specified number of time bins.
 * 
 * The number of frequency points used to calculate the discrete Fourier transforms is fft_size.
 * fft_size must be a power of 2 greater than 8 and less than MAX_FFT_POW2 as defined in spectrogram.h. 
 *  
 * The specified segment overlap must be an integer smaller than fft_size.   
 * Fs is the sampling frequency of the input signal specified in Hz.
 * The number of overlaping segments to average to form each spectral time bin is specified by navg. 
 * A value of navg=1 specified no time averaging.
 *
 * The averaged magnitude of the spectrogram is then saved in the psd structure (psd.magnitude) 
 * as a vector of length num_freq_bins * num_time_bins, where
 * num_freq_bins = (fft_size/2 + 1), 
 * num_time_bins = ((nsamps - overlap)/(fft_size - overlap)) / navg
 * 
 * The spectrogram function also defines frequency and time vectors (psd.freq and psd.time)
 * that specify the time (seconds) and frequency (Hz) of each spectrogram bins. 
 * psd.freq has length num_freq_bins and units of Hz.
 * psd.time has length num_time_bins and units of seconds.
 * 
 * The argument scale_method controls how the function will apply scaling
 * while computing a Fourier Transform. The available options are static
 * scaling (dividing the input at any stage by 2), dynamic scaling (dividing
 * the input at any stage by 2 if the largest absolute input value is greater or
 * equal than 0.5), or no scaling.
 * If static scaling is selected, the function will always scale intermediate
 * results, thus preventing overflow. The loss of precision increases in line
 * with fft_size and is more pronounced for input signals with a small magnitude
 * (since the output is scaled by 1/fft_size). To select static scaling,
 * set the argument scale_method to a value of 1. The block exponent
 * returned will be log2(fft_size) depending upon the number of times
 * that the function scales the intermediate set of results.
 * If dynamic scaling is selected, the function will inspect intermediate
 * results and only apply scaling where required to prevent overflow. The loss
 * of precision increases in line with the size of the FFT and is more pronounced
 * for input signals with a large magnitude (since these factors
 * increase the need for scaling). The requirement to inspect intermediate
 * results will have an impact on performance. To select dynamic scaling, set
 * the argument scale_method to a value of 2. The block exponent returned
 * will be between 0 and log2(fft_size).
 * If no scaling is selected, the function will never scale intermediate results.
 * There will be no loss of precision unless overflow occurs and in this case
 * the function will generate saturated results. The likelihood of saturation
 * increases in line with the fft_size and is more pronounced for input signals
 * with a large magnitude. To select no scaling, set the argument
 * scale_method to 3. The block exponent returned will be 0.
 * --------------------------------------------------------------------------------
 * Here's and example of how to use the spectrogram functions:
 * 
 * spectrogram_t psd;
 * int psd_fft_size = 256;		// fft size
 * int psd_overlap = 128;		// 50% fft overlap
 * int psd_time_average = 3;    // average 3 ffts segments for each time bin
 * int psd_scaling_method = 1;  // static scaling
 *
 * // initialize the spectrogram
 * if(spectrogram_init(&psd, psd_fft_size, psd_overlap, adc_fs) < 0) {
 *   log_printf("Error initializing spectrogram\n");
 *   return(0);
 * }
 * 
 * int16_t *buffer = ...  // read data into int16 buffer
 *
 * // build spectrogram from the adc data buffer
 * spectrogram(&psd, buffer, nsamps, psd_time_average, psd_scaling_method);
 * 
 * // save the spectrogram as a pgm image file
 * spectrogram_write_pgm(&psd, filename);
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
 * Embedded Ocean Systems (EOS), 2015
 *
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#include "spectrogram.h"


//
// calculate the spectrogram for the input signal
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__ComplexFFT.html
//
int spectrogram_f32(uint8_t *input, float32_t *output, uint16_t nsamps, uint16_t overlap)
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

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_f32(twid, buf, 0, 0);

		/* Calculating the magnitude at each bin */
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
// calculate the spectrogram for the input signal
// see https://arm-software.github.io/CMSIS_5/DSP/html/group__ComplexFFT.html
//
int spectrogram_q31(uint8_t *input, int32_t *output, uint16_t nsamps, uint16_t overlap)
{
	int k, m, n;
	uint16_t fft_size = PSD_FFT_SIZE;
	int32_t mag[PSD_FFT_SIZE]; // real
	int32_t buf[2*PSD_FFT_SIZE]; // complex

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
			buf[m] = ((int32_t)uv >> 8); // real
			buf[m+1] = 0; // imag
		}

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_q31(twid, buf, 0, 0);

		/* Calculating the magnitude at each bin */
		arm_cmplx_mag_q31(buf, mag, fft_size);
		
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


