/*
 * wav_file.c - wave data file I/O utilities
 * 
 * File names contain a user defined prefix and the date:
 * Raw data file names have a .flac extension (prefix_YYMMDD_HHMMSS.flac)
 * Header file names have a .txt extension (prefix_YYMMDD_HHMMSS.txt)
 *
 * ------
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
 * Embedded Ocean Systems (EOS), 2014
 */

#include <stdio.h>
#include <getopt.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
//#include <sys/statfs.h>

#include "wav_file.h"

extern int verbose_level;

/*
* The canonical WAVE format starts with the RIFF header:
* Offset  Size Name           Description
*------------------------------------------------------------------------------
* 0       4    ChunkID        Contains the letters "RIFF" in ASCII form
* 4       4    ChunkSize      4 + (8 + SubChunk1Size) + (8 + SubChunk2Size)
* 8       4    Format          Contains the letters "WAVE"
* 12      4    Subchunk1ID    Contains the letters "fmt "
* 16      4    Subchunk1Size  16 for PCM.  This is the size of the Subchunk that follows this number.
* 20      2    AudioFormat    PCM = 1 (i.e. Linear quantization)
* 22      2    NumChannels    Mono = 1, Stereo = 2, etc.
* 24      4    SampleRate     Hz
* 28      4    ByteRate       == SampleRate * NumChannels * BitsPerSample/8
* 32      2    BlockAlign     == NumChannels * BitsPerSample/8
* 34      2    BitsPerSample  8 bits = 8, 16 bits = 16, etc.
* 36      4    Subchunk2ID    Contains the letters "data"
* 40      4    Subchunk2Size  == NumSamples * NumChannels * BitsPerSample/8
* 44      *    Data           The actual sound data.
*/

size_t wav_read_header(wav_file_t *file)
{
  size_t nrd = 0;
  FILE *fp = file->fp;
  wav_file_header_t *wav = &file->hdr;
  fseek(fp, 0, SEEK_SET);  
  nrd = fread(wav->ChunkID,1,4,fp);
  nrd += fread(&(wav->ChunkSize),1,4,fp);
  nrd += fread(wav->Format,1,4,fp);
  nrd += fread(wav->Subchunk1ID,1,4,fp);
  nrd += fread(&wav->Subchunk1Size,1,4,fp);
  nrd += fread(&wav->AudioFormat,1,2,fp);
  nrd += fread(&wav->NumChannels,1,2,fp);
  nrd += fread(&wav->SampleRate,1,4,fp);
  nrd += fread(&wav->ByteRate,1,4,fp);
  nrd += fread(&wav->BlockAlign,1,2,fp);
  nrd += fread(&wav->BitsPerSample,1,2,fp);
  nrd += fread(wav->Subchunk2ID,1,4,fp);
  nrd += fread(&wav->Subchunk2Size,1,4,fp);
  //nrd = fread(wav, 1, 44, fp);
  return(nrd);
}

size_t wav_write_header(wav_file_t *file)
{
  size_t nwrt = 0;
  FILE *fp = file->fp;
  wav_file_header_t *wav = &file->hdr;
  fseek(fp, 0, SEEK_SET);  
  nwrt = fwrite(wav->ChunkID,1,4,fp);
  nwrt += fwrite(&wav->ChunkSize,1,4,fp);
  nwrt += fwrite(wav->Format,1,4,fp);
  nwrt += fwrite(wav->Subchunk1ID,1,4,fp);
  nwrt += fwrite(&wav->Subchunk1Size,1,4,fp);
  nwrt += fwrite(&wav->AudioFormat,1,2,fp);
  nwrt += fwrite(&wav->NumChannels,1,2,fp);
  nwrt += fwrite(&wav->SampleRate,1,4,fp);
  nwrt += fwrite(&wav->ByteRate,1,4,fp);
  nwrt += fwrite(&wav->BlockAlign,1,2,fp);
  nwrt += fwrite(&wav->BitsPerSample,1,2,fp);
  nwrt += fwrite(wav->Subchunk2ID,1,4,fp);
  nwrt += fwrite(&wav->Subchunk2Size,1,4,fp); 
  //nwrt = fwrite(wav, 1, 44, fp);
  return(nwrt);
}

void wav_print_header(wav_file_t *file)
{
  char str[5];
  wav_file_header_t *wav = &file->hdr;
  strncpy(str, wav->ChunkID,4); str[4]=0;
  fprintf(stdout, "ChunkID:        %s\n", str); 
  fprintf(stdout, "ChunkSize:      %d\n", wav->ChunkSize);
  strncpy(str, wav->Format,4); str[4]=0;
  fprintf(stdout, "Format:         %s\n", str);
  strncpy(str, wav->Subchunk1ID,4); str[4]=0;
  fprintf(stdout, "Subchunk1ID:    %s\n", str);
  fprintf(stdout, "Subchunk1Size:  %d\n", wav->Subchunk1Size);
  fprintf(stdout, "AudioFormat:    %d\n", wav->AudioFormat);
  fprintf(stdout, "NumChannels:    %d\n", wav->NumChannels);
  fprintf(stdout, "SampleRate:     %d\n", wav->SampleRate);
  fprintf(stdout, "ByteRate:       %d\n", wav->ByteRate);
  fprintf(stdout, "BlockAlign:     %d\n", wav->BlockAlign);
  fprintf(stdout, "BitsPerSample:  %d\n", wav->BitsPerSample);
  strncpy(str, wav->Subchunk2ID,4); str[4]=0;
  fprintf(stdout, "Subchunk2ID:    %s\n", str);
  fprintf(stdout, "Subchunk2Size:  %d\n", wav->Subchunk2Size);
}

//
// initialize wav header with default values
//
void wav_init_header(wav_file_t *file, int nbps, int fs, int nchans)
{
  wav_file_header_t *wav = &file->hdr;
  wav->ChunkID[0] = 'R';
  wav->ChunkID[1] = 'I';
  wav->ChunkID[2] = 'F';
  wav->ChunkID[3] = 'F';
  wav->ChunkSize = 36; // = 4 + 8 + 16 + 8 + 0;
  wav->Format[0] = 'W';
  wav->Format[1] = 'A'; 
  wav->Format[2] = 'V';
  wav->Format[3] = 'E'; 
  wav->Subchunk1ID[0] = 'f';
  wav->Subchunk1ID[1] = 'm';
  wav->Subchunk1ID[2] = 't';
  wav->Subchunk1ID[3] = ' ';  // space, not null
  wav->Subchunk1Size = 16;
  wav->AudioFormat = 1;
  wav->NumChannels = nchans;
  wav->SampleRate = fs;
  wav->ByteRate = fs * nchans * nbps / 8;
  wav->BlockAlign = nchans * nbps / 8;
  wav->BitsPerSample = nbps;
  wav->Subchunk2ID[0] = 'd';
  wav->Subchunk2ID[1] = 'a';
  wav->Subchunk2ID[2] = 't';
  wav->Subchunk2ID[3] = 'a';
  wav->Subchunk2Size = 0;  // no data yet
}

//
// wave_fclose
//
void wav_fclose(wav_file_t *file)
{
   fclose(file->fp);
   free(file);
}

int wav_set_description(wav_file_t *file, char *str) 
{
	if(str == NULL) return(0);
	strncpy(str, file->description, sizeof(file->description));
    return(1);
}

// 
// Open and initialize a new wav file using the time as the file name.
//
wav_file_t *wav_fopen(char *name, char *type, int nbps, int fs, int nchans)
{
   wav_file_t *file = malloc(sizeof(wav_file_t));

   if(file == NULL) {
     fprintf( stdout, "wav_fopen: invalid file objects\n");
     return(0);
   }

   sprintf(file->name, "s", name);

   // Create an empty file for output operations. 
   // If a file with the same name already exists, its contents are discarded 
   // and the file is treated as a new empty file.
   file->fp = fopen(file->name, type);
   if(file->fp == NULL) {
      fprintf( stdout, "wav_fopen: Can't open output file\n");
      return(file);
   }

   // if the file is for writing
   if((type[0] == 'w')||(type[0]=='a')) {
     
	 // initialize the wav header
     wav_init_header(file, nbps, fs, nchans);
     // write wav header
     if(wav_write_header(file) <= 0) {
       fprintf( stdout, "wav_fopen: error writing wav header\n");
     }
   
   } 
   // if the file is for reading
   else if( type[0] == 'r' ) {
     
	 // read the wav header
     wav_read_header(file);
   
   }
    
   return(file);
}

/*
 * Write data buffer to file using the specified number of samples and bytes per sample.
 * Opens and closes the file on each write call.
 */
size_t wav_write(wav_file_t *file, uint8_t *data, uint32_t nsamps)
{
   if(file->fp == NULL) {
      fprintf(stdout, "wav_write: output file not openned, no data written\n");
      return(-1);
   }

   // read wav header
   size_t nrd = wav_read_header(file);
   if(nrd < 44) {
      fprintf(stdout, "wav_write: error reading wav header\n");
   }

   // check nbps
   int nbps = file->nbps;
   int nchans = file->nchans;
   if(nbps != file->hdr.BitsPerSample) {
      fprintf(stdout, "wav_write: inconsistent data format\n");
      return(-1);
   }
   
   // seek to end of file to append data
   fseek(file->fp, 0, SEEK_END);

   // write data buffer
   size_t nwrt = fwrite(data, nbps/8, nsamps, file->fp);

   if(nwrt < (size_t)nsamps) {
      fprintf(stdout, "wav_write: error writing data to %s\n", file->name);
	  nsamps = (int)nwrt;
   }

   // update the header:
   //  - Subchunk2Size - add number of bytes in this buffer
   //  - ChunkSize - for PCM, 36 + Subchunk2Size.
   file->hdr.Subchunk2Size += (int)(nsamps * nchans * nbps/8); // add number of bytes 
   file->hdr.ChunkSize = 36 + file->hdr.Subchunk2Size;
   
   // rewrite the header
   nwrt = wav_write_header(file);
   if(nwrt < 44) {
      fprintf(stdout, "wav_write: error writing wav header\n");
   }
   //wav_print_header(fp, &file->hdr);   

   // force output to file
   fflush(file->fp);

   return(nwrt);
}




