/**
 * Spectrum test main 
 * Main program to test computation and timing of spectrum.c functions using synthetic data.
 * Output is printed to stdout in a format that can be cut and pastes into matlab.
 * Uses systick interrupt to measure function execution time.
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "wispr.h"
#include "board.h"
#include "console.h"
#include "spectrum.h"
//#include "arm_math.h"
//#include "arm_const_structs.h"

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_MAX_BUFFER_SIZE+1];
uint8_t *adc_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t adc_header;

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_MAX_BUFFER_SIZE+1];
float32_t *psd_data = (float32_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t psd_header;

COMPILER_WORD_ALIGNED int32_t test_data[ADC_MAX_SAMPLES_PER_BUFFER+1];

void gen_test_signal(int32_t *buf, uint16_t nsamps, uint32_t freq, uint32_t fs, float32_t amp, float32_t noise);

volatile uint32_t ul_ms_ticks = 0;
void SysTick_Handler(void)
{
	ul_ms_ticks++;
}

//
// main
//
int main (void)
{		
	// initialize the board specific functions (clocks, gpio, console, wdt, ...)
	board_init();

	// set defaults for spectral estimation
	uint16_t nfft = PSD_MAX_FFT_SIZE;
	uint16_t nsamps = ADC_MAX_SAMPLES_PER_BUFFER;
	uint32_t fs = 100000;
	float32_t amp = 1.0;
	float32_t stdev = 0.01;
	uint32_t fc = 12000;
	float32_t scaling = 1024.0;

	uint16_t nbins = nfft/2;
	uint16_t overlap = 0;
	uint8_t nbps = 3;

	uint32_t tick1, tick2, ms1, ms2;

	/* Setup SysTick Timer for 1 msec interrupts */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}

	tick1 = ul_ms_ticks;
	delay_ms(1);
	tick2 = ul_ms_ticks;
	printf("Calibration elapse ticks: %d\r\n", tick2 - tick1) ; // 8 ticks are used to read the register twice and subtract.

	while(1) {

		nfft = console_prompt_uint16("nfft = ", nfft, 10);
		nsamps = console_prompt_uint16("nsamps = ", nsamps, 10);
		fs = console_prompt_uint32("fs = ", fs, 10);
		fc = console_prompt_uint32("fc = ", fc, 10);
		amp = console_prompt_f32("amp = ", amp, 10);
		stdev = console_prompt_f32("noise stdev = ", stdev, 10);
		scaling = console_prompt_f32("scaling = ", scaling, 10);

		adc_header.samples_per_block = nsamps;
		adc_header.sampling_rate = fs;
		adc_header.sample_size = nbps;
		adc_header.block_size = ADC_MAX_BUFFER_SIZE;
		
		// test signal
		gen_test_signal(test_data, nsamps, fc, fs, amp, stdev);

		// copy data into adc buffer 
		uint8_t *buf = (uint8_t *)test_data;
		int m = 0;
		for(uint16_t n = 0; n < nsamps; n++) {
			for(uint8_t k = 0; k < nbps; k++) {
				adc_data[m++] = buf[4*n+k];
			}
		}
			
		// test f32 spectrum
		spectrum_init_f32(&nbins, nfft, overlap, fs, nbps, HANN_WINDOW);		
		tick1 = ul_ms_ticks;
		spectrum_f32(&psd_header, psd_data, &adc_header, adc_data, nsamps);
		tick2 = ul_ms_ticks;
		ms1 = tick2 - tick1;

		printf("\r\n");
		printf("nfft = %d;\r\n", nfft);
		printf("nsamps = %d;\r\n", nsamps);
		printf("fs = %d;\r\n", fs);
		printf("fc = %d;\r\n", fc);
		printf("amp = %.2f;\r\n", amp);
		printf("stdev = %.2f;\r\n", stdev);

		// for testing only
		printf("\r\n");
		printf("psd_f32 = [\r\n");
		for(int n = 0; n < nbins; n++) {
			printf("%f ", psd_data[n]);
		}
		printf("];\r\n");
		
		// test q31 spectrum
		spectrum_init_q31(&nbins, nfft, overlap, fs, nbps, HANN_WINDOW);
		tick1 = ul_ms_ticks;
		spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, nsamps);
		tick2 = ul_ms_ticks;
		ms2 = tick2 - tick1;

		// for testing only
		printf("psd_q31 = [\r\n");
		for(int n = 0; n < nbins; n++) {
			printf("%f ", psd_data[n]);
		}
		printf("];\r\n");

		float32_t df = (float32_t)fs / (float32_t)nfft;
		float32_t f = 0.0;
		printf("freq = [\r\n");
		for(int n = 0; n < nbins; n++) {
			printf("%.1f ", f);
			f += df;
		}
		printf("];\r\n");

		printf("\r\n");
		printf("spectrum_f32_msec = %d;\r\n", ms1) ;
		printf("spectrum_q31_msec = %d;\r\n", ms2) ;

	}

	printf("Finished\n\r");
	
	exit(0);
}

void gen_test_signal(int32_t *buf, uint16_t nsamps, uint32_t freq, uint32_t fs, float32_t amp, float32_t stdev)
{	
	float32_t max_value = 8388607.0 / ADC_SCALING; // 2^23
	float32_t w = 2.0 * PI * (float32_t)freq;
	float32_t dt = 1.0 / (float32_t)fs;
	float32_t A = 1.73205080; // sqrt(12/4)
	for(int n = 0; n < nsamps; n++) {
		// uniform random noise with zero mean and variance = sqrt(stdev)
		int rv = rand();
		float32_t noise = stdev * A * (2.0 *((float32_t)rv / (float32_t)RAND_MAX) - 1.0f);
		//float32_t noise = stdev * A * (2.0 *((float32_t)rand() / (float32_t)RAND_MAX) - 1.0f);
		float32_t t = (float32_t)n * dt;
		float32_t x =  (noise + amp*arm_sin_f32(w*t));
		buf[n] = (int32_t)(max_value * x);
	}
/*
	printf("test_sig = [\r\n");
	for(int n = 0; n < 1024; n++) {
		//printf("%f ", (float32_t)buf[n] / max_value);
		printf("%d ", buf[n]);
	}
	printf("];\r\n");
*/
}
