/**
 * Spectrum test main
 * Data is read from a test file saved on the sd card.
 * Can also generates synthetic data to test the spectrum functions.
 * Output is printed to stdout in matlab format.
 * Function execution time is found using the systick interface.
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "wispr.h"
#include "board.h"
#include "console.h"

#include "spectrum.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "sd_card.h"

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_MAX_BUFFER_SIZE+1];
uint8_t *adc_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t adc_header;

float32_t psd_data[PSD_MAX_FFT_SIZE+1]; // data follows header
wispr_data_header_t psd_header;

uint16_t nbins;
uint16_t overlap = 0;
uint8_t nbps;

uint16_t nfft = PSD_MAX_FFT_SIZE;
uint16_t nsamps = ADC_MAX_SAMPLES_PER_BUFFER;
uint32_t fs = 100000;
float32_t amp = 1.0;
float32_t stdev = 0.01;
uint32_t fc = 12000;
float32_t scaling = 1024.0;

fat_file_t ff;
int count;

void read_test_data(void);
void generate_test_data(void);

volatile uint32_t ul_ms_ticks = 0;
void SysTick_Handler(void)
{
	ul_ms_ticks++;
}

int main (void)
{
	int go = 1;
		
	// initialize the board specific functions (clocks, gpio, console, wdt, ...)
	// returns the reason the board was last reset (user, sleep, watchdog, ...)
	int reset_type = board_init();

	/* Setup SysTick Timer for 1 msec interrupts */
	uint32_t tick1, tick2, ms1, ms2;
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}

	tick1 = ul_ms_ticks;
	delay_ms(1);
	tick2 = ul_ms_ticks;
	printf("Calibration elapse ticks: %d\r\n", tick2 - tick1) ; // 8 ticks are used to read the register twice and subtract.

	sd_card_mount_fat(1);
	ff.state = SD_FILE_CLOSED;
	
	int do_read_test_data = console_prompt_int("Read test from file", 1, 10);
	
	nfft = console_prompt_uint16("nfft = ", nfft, 10);
	
	go = 1;
	count = 1;
	while(go) {

		if(do_read_test_data) {
			read_test_data();
		} else {
			generate_test_data();
		}
		
		// test f32 spectrum
		spectrum_init_f32(&nbins, nfft, overlap, HANN_WINDOW);		
		
		tick1 = ul_ms_ticks;

		spectrum_f32(&psd_header, psd_data, &adc_header, adc_data, nsamps);

		tick2 = ul_ms_ticks;
		ms1 = tick2 - tick1;

		printf("\r\n");
		printf("nfft = %d;\r\n", nfft);
		printf("nsamps = %d;\r\n", nsamps);
		printf("fs = %d;\r\n", fs);
		printf("nbps = %d;\r\n", nbps);

		printf("\r\n");
		printf("psd_f32 = [");
		for(int n = 0; n < nbins; n++) {
			printf("%f ", psd_data[n]);
		}
		printf("];\r\n");
		printf("psd_f32_msec = %d;\r\n", ms1) ;
		
		// test q31 spectrum
		spectrum_init_q31(&nbins, nfft, overlap, HANN_WINDOW);
		tick1 = ul_ms_ticks;
		spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, nsamps);
		tick2 = ul_ms_ticks;
		ms2 = tick2 - tick1;

		// for testing only
		printf("psd_q31 = [");
		for(int n = 0; n < nbins; n++) {
			printf("%f ", psd_data[n]);
		}
		printf("];\r\n");
		printf("psd_q31_msec = %d;\r\n", ms2) ;

		float32_t df = (float32_t)fs / (float32_t)nfft;
		float32_t f = 0.0;
		printf("freq = [");
		for(int n = 0; n < nbins; n++) {
			printf("%.1f ", f);
			f += df;
		}
		printf("];\r\n");
		printf("\r\n");

		go = console_prompt_int("Process another buffer", go, 10);
		count++;
	
	}

	//printf("Finished\n\r");
	sd_card_close_fat(&ff);
	
	exit(0);
}

void read_test_data(void)
{
	int go = 1;
	int new_file = console_prompt_int("Open new test file", 1, 10);
	
	if(new_file) {
		char filename[32];
		sprintf(filename, "test%d.dat", count);
		while(go) {
			char str[32];
			printf("Enter test file [%s]: ", filename);
			if( console_input(str, 32, 20) > 0) strncpy(filename, str, 32);
			sd_card_close_fat(&ff);
			FRESULT res = sd_card_open_fat(&ff, filename, FA_OPEN_EXISTING | FA_READ, 1);
			if(res != FR_OK ) {
				printf("Error opening test data file: %s\r\n", filename);
			} else {
				go = 0;
			}
		}
	}
	
	// Read data buffer header
	UINT nrd, nbytes;
	nbytes = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	FRESULT res = f_read(&ff.file, adc_buffer, nbytes, &nrd);
	if (nrd != nbytes) {
		printf("failed to read data buffer: %d\n", nrd);
		return;
	}

	// Parse data header
	if( wispr_parse_data_header(adc_buffer, &adc_header) == 0 ) {
		fprintf(stdout, "failed to parse data header\n");
		return;
	}
	    
	nsamps = adc_header.samples_per_block;
	fs = adc_header.sampling_rate;
	nbps = adc_header.sample_size;
	nbins = nfft/2;
	overlap = 0;
}

void generate_test_data(void)
{
	nfft = console_prompt_uint16("nfft = ", nfft, 10);
	nsamps = console_prompt_uint16("nsamps = ", nsamps, 10);
	nbps = console_prompt_uint8("sample size = ", nbps, 10);
	fs = console_prompt_uint32("fs = ", fs, 10);
	fc = console_prompt_uint32("fc = ", fc, 10);
	amp = console_prompt_f32("amp = ", amp, 10);
	stdev = console_prompt_f32("noise stdev = ", stdev, 10);
	//scaling = console_prompt_f32("scaling = ", scaling, 10);

	adc_header.samples_per_block = nsamps;
	adc_header.sampling_rate = fs;
	adc_header.sample_size = nbps;
	adc_header.block_size = ADC_MAX_BUFFER_SIZE;
		
	int32_t *buf = (int32_t *)adc_data;
	float32_t max_value = 8388607.0 / ADC_SCALING; // 2^23
	float32_t w = 2.0 * PI * (float32_t)fc;
	float32_t dt = 1.0 / (float32_t)fs;
	float32_t A = 1.73205080; // sqrt(12/4)
	for(int n = 0; n < nsamps; n++) {
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

