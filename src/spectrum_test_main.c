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
#include <math.h>

#include "wispr.h"
#include "board.h"
#include "console.h"

#include "spectrum.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "sd_card.h"

COMPILER_WORD_ALIGNED uint8_t adc_buffer[ADC_MAX_BUFFER_SIZE+4];
uint8_t *adc_data = &adc_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t adc_header;

COMPILER_WORD_ALIGNED uint8_t psd_buffer[PSD_MAX_BUFFER_SIZE+4];
float32_t *psd_data = (float32_t *)&psd_buffer[WISPR_DATA_HEADER_SIZE]; // data follows header
wispr_data_header_t psd_header;

uint16_t nbins;
uint16_t overlap = 0;

uint16_t nfft = PSD_MAX_FFT_SIZE;
uint8_t nbps = 2;
uint16_t nsamps = ADC_MAX_SAMPLES_PER_BUFFER;
uint32_t fs = 50000;
float32_t amp = 1.0;
float32_t stdev = 0.01;
uint32_t fc = 1000;
float32_t scaling = 1.0;
float32_t vref = 5.0;

fat_file_t ffin;
fat_file_t ffout;
fat_file_t ffout2;
uint16_t count;

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
	uint32_t tick1, tick2;
	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}
	
	tick1 = ul_ms_ticks;
	delay_ms(1);
	tick2 = ul_ms_ticks;
	printf("Calibration elapse ticks: %d\r\n", tick2 - tick1) ; // 8 ticks are used to read the register twice and subtract.
	
	sd_card_mount_fat(1);
	ffin.state = SD_FILE_CLOSED;
	ffout.state = SD_FILE_CLOSED;
	
	int file_count = 1;
	uint8_t use_q31 = 0;
	int do_read_from_file = 1;

	while(1)
	{
		
	do_read_from_file = console_prompt_int("Read test data from file", 1, 10);
	if(do_read_from_file == 0) break;
	
	FRESULT res;
	char filename[32];
	char str[32];
	sprintf(filename, "test%d.dat", file_count);
	if(do_read_from_file) {
		go = 1;
		while(go) {
			printf("Enter input data file [%s]: ", filename);
			if( console_input(str, 32, 20) > 0) strncpy(filename, str, 32);
			res = sd_card_open_fat(&ffin, filename, FA_OPEN_EXISTING | FA_READ, 1);
			if(res != FR_OK ) {
				printf("Error opening test data file: %s\r\n", filename);
			} else {
				go = 0;
			}
		}
	}
	
	sprintf(filename, "test%d.psd", file_count);
	printf("\r\nEnter output data file [%s]: ", filename);
	if( console_input(str, 32, 20) > 0) strncpy(filename, str, 32);

	tick1 = ul_ms_ticks;
	res = sd_card_open_fat(&ffout, filename, FA_CREATE_ALWAYS | FA_WRITE, 1);
	tick2 = ul_ms_ticks;	
	printf("sd_card_open_fat: msec %d\r\n", tick2 - tick1);

	sprintf(filename, "out%d.dat", file_count);
	tick1 = ul_ms_ticks;
	res = sd_card_open_fat(&ffout2, filename, FA_CREATE_ALWAYS | FA_WRITE, 1);
	tick2 = ul_ms_ticks;
	printf("sd_card_open_fat: msec %d\r\n", tick2 - tick1);
	
	if( res == FR_OK ) {
		printf("\r\nOpen new psd file: %s\r\n", filename);
	} else {
		printf("\r\nError opening new psd file: %s\r\n", filename);		
	}

	nfft = console_prompt_uint16("Enter nfft = ", nfft, 10);

	use_q31 = console_prompt_uint8("Use q31 ffts", use_q31, 10);

	uint8_t wintype = RECT_WINDOW;
	wintype = console_prompt_uint8("Enter wintype = ", wintype, 10);	
	printf("\r\n");

	go = 1;
	count = 1;
	while(go) {

		if(do_read_from_file == 1) {
			read_test_data();
		} else 	if(do_read_from_file == 2) {
			generate_test_data();
		} else {
			break;
		}
		
		if(use_q31) {
			spectrum_init_q31(&nbins, nfft, overlap, wintype);
		} else {
			spectrum_init_f32(&nbins, nfft, overlap, wintype);
		}
		
		tick1 = ul_ms_ticks;
	
		if(use_q31) {
			spectrum_q31(&psd_header, psd_data, &adc_header, adc_data, nsamps);			
		} else {
			spectrum_f32(&psd_header, psd_data, &adc_header, adc_data, nsamps);		
		}
		
		tick2 = ul_ms_ticks;
		uint32_t msec = tick2 - tick1;
		
		if(nbps == 2) scaling = vref / 32767.0;
		if(nbps == 3) scaling = vref / 8388607.0;
		
		if( 0 ) {
			printf("\r\n");
			printf("nfft_%d = %d;\r\n", count, nfft);
			printf("nsamps_%d = %d;\r\n", count, nsamps);
			printf("fs_%d = %d;\r\n", count, fs);
			printf("nbps_%d = %d;\r\n", count, nbps);
			printf("scaling_%d = %.10f;\r\n", count, scaling);
			printf("out_%d = [", count);
			for(int n = 0; n < nbins; n++) {
				printf("%f ", psd_data[n]);
			}
			printf("];\r\n");
		}
		
		if(use_q31) {
			printf("spectrum_q31_execution_msec = %d;\r\n", msec) ;
		} else {
			printf("spectrum_f32_execution_msec = %d;\r\n", msec) ;
		}
		
		// serialize the buffer header - write the latest adc header onto the front of the buffer
		wispr_serialize_data_header(&psd_header, psd_buffer);

		// write the adc buffer - both header and data
		
		tick1 = ul_ms_ticks;
		if( sd_card_write_fat(&ffout2, adc_buffer, ADC_MAX_BLOCKS_PER_BUFFER) != FR_OK ) {
			printf("Error writing to file: %s\r\n", ffout.name);
		}
		tick2 = ul_ms_ticks;
		printf("sd_card_write_fat: msec %d\r\n", tick2 - tick1);

		uint16_t psd_write_size = PSD_MAX_BLOCKS_PER_BUFFER;
		if( sd_card_write_fat(&ffout, psd_buffer, psd_write_size) != FR_OK ) {
			printf("Error writing to file: %s\r\n", ffout.name);
		}

		go = console_prompt_int("Process another buffer", go, 10);
		count++;
	
	}

	sd_card_close_fat(&ffin);
	sd_card_close_fat(&ffout);
	
	tick1 = ul_ms_ticks;
	sd_card_close_fat(&ffout2);
	tick2 = ul_ms_ticks;
	printf("sd_card_close_fat: msec %d\r\n", tick2 - tick1);
	
	file_count++;
	
	}
	printf("Finished\n\r");
	
	exit(0);
}

void read_test_data(void)
{
	// Read data buffer header
	UINT nrd, nbytes;
	nbytes = ADC_MAX_BLOCKS_PER_BUFFER * SD_MMC_BLOCK_SIZE;
	FRESULT res = f_read(&ffin.file, adc_buffer, nbytes, &nrd);
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

	//printf("\r\n");
	//printf("nfft_%d = %d;\r\n", count, nfft);
	//printf("nsamps = %d;\r\n", nsamps);
	//printf("fs = %d;\r\n", fs);
	//printf("nbps_%d = %d;\r\n", count, nbps);

	//printf("sig_%d = [", count);
	//for(int n = 0; n < 1024; n++) {
	//	//printf("%f ", (float32_t)buf[n] / max_value);
	//	printf("%d ", adc_data[n]);
	//}
	//printf("];\r\n");

}

void generate_test_data(void)
{
	nsamps = console_prompt_uint16("nsamps = ", nsamps, 10);
	nbps = console_prompt_uint8("sample size = ", nbps, 10);
	fs = console_prompt_uint32("fs = ", fs, 10);
	fc = console_prompt_uint32("fc = ", fc, 10);
	amp = console_prompt_f32("amp = ", amp, 10);
	stdev = console_prompt_f32("noise stdev = ", stdev, 10);
	//scaling = console_prompt_f32("scaling = ", scaling, 10);
	nbins = nfft/2;
	overlap = 0;

	adc_header.samples_per_block = nsamps;
	adc_header.sampling_rate = fs;
	adc_header.sample_size = nbps;
	adc_header.block_size = ADC_MAX_BUFFER_SIZE;

	uint8_t *buf = (uint8_t *)adc_data;
	float32_t max_value = 8388607.0 / ADC_SCALING; // 2^23
	float32_t w = 2.0 * PI * (float32_t)fc;
	float32_t dt = 1.0 / (float32_t)fs;
	float32_t A = 1.73205080; // sqrt(12/4)
	int m = 0;
	for(int n = 0; n < nsamps; n++) {
		int rv = rand();
		float32_t noise = stdev * A * (2.0 *((float32_t)rv / (float32_t)RAND_MAX) - 1.0f);
		//float32_t noise = stdev * A * (2.0 *((float32_t)rand() / (float32_t)RAND_MAX) - 1.0f);
		float32_t t = (float32_t)n * dt;
		float32_t x =  (noise + amp*arm_sin_f32(w*t));
		int32_t v = (int32_t)(max_value * x);
		if(nbps == 2) {
			buf[m++] = (uint8_t)((v & 0x00FF0000) >> 16);
			buf[m++] = (uint8_t)((v & 0xFF000000) >> 24);
		} else if(nbps == 3) {
			buf[m++] = (uint8_t)((v & 0x0000FF00) >> 8);
			buf[m++] = (uint8_t)((v & 0x00FF0000) >> 16);
			buf[m++] = (uint8_t)((v & 0xFF000000) >> 24);
		}
	}
	
	//printf("sig_%d = [", count);
	//for(int n = 0; n < 1024; n++) {
	//	//printf("%f ", (float32_t)buf[n] / max_value);
	//	printf("%d ", adc_data[n]);
	//}
	//printf("];\r\n");

}

