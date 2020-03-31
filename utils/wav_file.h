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
   char name[128];        // current data file name
   char description[128];        // current data file name
   uint32_t sec;         // second start time
   uint32_t usec;        // usecond start time
   int nchans;            // number of channels
   int nbps;              // number bits per sample
   int fs;                // sampling rate (hertz)
   uint32_t nsamps;      // number of samples per channel in file
   wav_file_header_t hdr;
} wav_file_t;

/* function prototypes */
extern size_t wav_read_header(wav_file_t *file);
extern size_t wav_write_header(wav_file_t *file);
extern void wav_print_header(wav_file_t *file);
extern void wav_init_header(wav_file_t *file, int nbps, int fs, int nchans);
extern void wav_close(wav_file_t *file);
extern wav_file_t *wav_open(char *name, char *type, int nbps, int fs, int nchans);
extern size_t wav_write(wav_file_t *file, uint8_t *data, uint32_t nsamps);
extern int wav_set_description(wav_file_t *file, char *str);

#endif /* _WAV_file_H */

