#
# Makefile 
#

CC=gcc

LIBS = 
INCS = -I..\src
CFLAGS = -W -O2 

SRCS = wav_file.c \
 ..\src\wispr.c \
 ..\src\epoch.c

OBJS = $(SRCS:.c=.o)

.c.o:
	$(CC) $(CFLAGS) $(INCS) -c $<  -o $@

all:
	make dat_dump_sdcard
	make psd_dump_sdcard
	make wav_dump_sdcard
	
dat_dump_sdcard: dat_dump_sdcard.c $(OBJS)
	$(CC) dat_dump_sdcard.c -o dat_dump_sdcard $(CFLAGS) $(INCS) $(OBJS)

psd_dump_sdcard: psd_dump_sdcard.c $(OBJS)
	$(CC) psd_dump_sdcard.c -o psd_dump_sdcard $(CFLAGS) $(INCS) $(OBJS)

wav_dump_sdcard:  wav_dump_sdcard.c $(OBJS)
	$(CC) wav_dump_sdcard.c -o wav_dump_sdcard $(CFLAGS) $(INCS) $(OBJS)

clean:
	rm $(OBJS)
	rm dat_dump_sdcard
	rm psd_dump_sdcard
	rm wav_dump_sdcard


mingw_clean:
	del *.o
	del dat_dump_sdcard
	del wav_dump_sdcard
