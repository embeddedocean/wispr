/*
*  wav_file.h:  data file utility
*/
#ifndef _WAV_FILE_H
#define _WAV_FILE_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>

#define WAVE_FORMAT_PCM	0x0001	// PCM integer
#define WAVE_FORMAT_IEEE_FLOAT 0x0003	 // IEEE float32
#define WAVE_FORMAT_ALAW 0x0006	 //	8-bit ITU-T G.711 A-law
#define WAVE_FORMAT_MULAW 0x0007 //	8-bit ITU-T G.711 µ-law
#define WAVE_FORMAT_EXTENSIBLE 0xFFFE // Determined by SubFormat

// standard/simple wav file header
typedef struct wav_file_header {
  char ChunkID[4];
  int ChunkSize;
  char Format[4];
  char Subchunk1ID[4];
  int Subchunk1Size;
  short AudioFormat;
  short NumChannels;
  int SampleRate;
  int ByteRate;
  short BlockAlign;
  short BitsPerSample;
  char Subchunk2ID[4];
  int Subchunk2Size;
} wav_file_header_t;

//  wav file type
typedef struct {
   FILE *fp;
   int nchans;            // number of channels
   int nbps;              // number bits per sample
   int fs;                // sampling rate (hertz)
   uint16_t format;
   uint32_t nsamps;       // number of samples per channel in file
   wav_file_header_t hdr;
} wav_file_t;

/* function prototypes */
extern size_t wav_read_header(wav_file_t *file);
extern size_t wav_write_header(wav_file_t *file);
extern void wav_print_header(wav_file_t *file);
extern void wav_init_header(wav_file_t *file, int nbps, int fs, int nchans, short format);
extern void wav_close(wav_file_t *file);
extern wav_file_t *wav_open(char *name, char *type, int nbps, int fs, int nchans, short format);
extern size_t wav_write(wav_file_t *file, uint8_t *data, uint32_t nsamps);

#endif /* _WAV_file_H */

