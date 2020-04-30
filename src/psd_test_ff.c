/**
 * Spectrum test main
 * Generates synthetic data to test and time the spectrum functions.
 * Output is printed to stdout in matlab format.
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

//COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_MAX_BUFFE+1];
float32_t psd_data[PSD_MAX_FFT_SIZE+1]; // data follows header
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
	wispr_config_t wispr; // current configuration
		
	// initialize the board specific functions (clocks, gpio, console, wdt, ...)
	// returns the reason the board was last reset (user, sleep, watchdog, ...)
	int reset_type = board_init();

	uint16_t nfft = 1024;
	uint16_t nsamps;
	uint32_t fs;
	uint16_t nbins;
	uint16_t overlap = 0;
	uint8_t nbps;

	uint32_t tick1, tick2, ms1, ms2;

	/* Setup SysTick Timer for 1 msec interrupts */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}

	tick1 = ul_ms_ticks;
	delay_ms(1);
	tick2 = ul_ms_ticks;
	printf("Calibration elapse ticks: %d\r\n", tick2 - tick1) ; // 8 ticks are used to read the register twice and subtract.

	wispr.active_sd_card = 1;
	sd_card_mount_fat(1);

	fat_file_t ff;
	
	char filename[] = "psd_test1.dat";

	int go = 1;
	while(go) {
		char str[32];
		printf("Enter test file [%s]: ", filename);
		if( console_input(str, 32, 20) > 0) strncpy(filename, str, 32);	
		FRESULT res = sd_card_open_fat(&ff, filename, FA_OPEN_EXISTING | FA_READ, wispr.active_sd_card);
		if(res != FR_OK ) {
			printf("Error opening test data file: %s\r\n", filename);
		} else {
			go = 0;
		}
	}

	nfft = console_prompt_uint16("nfft = ", nfft, 10);
	
	UINT nrd, nbytes;
	
	go = 1;
	while(go) {

	    // Read data buffer header
		nbytes = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	    FRESULT res = f_read(&ff.file, adc_buffer, nbytes, &nrd);
	    if (nrd != nbytes) {
		    printf("failed to read data buffer: %d\n", nrd);
		    return -1;
	    }

		// Parse data header
		if( wispr_parse_data_header(adc_buffer, &adc_header) == 0 ) {
		    fprintf(stdout, "failed to parse data header\n");
		    return -1;
		    //continue;
		}
		
		nsamps = adc_header.samples_per_block;
		fs = adc_header.sampling_rate;
		nbps = adc_header.sample_size;
		nbins = nfft/2;
		overlap = 0;

		printf("\r\n");
		printf("nfft = %d;\r\n", nfft);
		printf("nsamps = %d;\r\n", nsamps);
		printf("fs = %d;\r\n", fs);
		printf("nbps = %d;\r\n", nbps);

		// test f32 spectrum
		spectrum_init_f32(&nbins, nfft, overlap, HANN_WINDOW);		
		
		tick1 = ul_ms_ticks;

		spectrum_f32(&psd_header, psd_data, &adc_header, adc_data, nsamps);

		tick2 = ul_ms_ticks;
		ms1 = tick2 - tick1;

		// for testing only
		printf("\r\n");
		printf("psd_f32 = [\r\n");
		for(int n = 0; n < nbins; n++) {
			printf("%f ", psd_data[n]);
		}
		printf("];\r\n");

		printf("\r\n");
		printf("psd_f32_msec = %d;\r\n", ms1) ;
		
		// test q31 spectrum
		spectrum_init_q31(&nbins, nfft, overlap, HANN_WINDOW);
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
		printf("psd_q31_msec = %d;\r\n", ms2) ;

		go = 0;
	}

	//printf("Finished\n\r");
	sd_card_close_fat(&ff);
	
	exit(0);
}

void gen_test_signal(int32_t *buf, uint16_t nsamps, uint32_t freq, uint32_t fs, float32_t amp, float32_t stdev)
{	
	float32_t max_value = 8388607.0 / ADC_SCALING; // 2^23
	float32_t w = 2.0 * PI * (float32_t)freq;
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
