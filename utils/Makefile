#
# Makefile 
#

CC=gcc

LIBS = 
INCS = -I..\atmel\wispr_v2.0\wispr_v2.0\src
CFLAGS = -W -O2 

SRCS = wav_file.c \
 ..\atmel\wispr_v2.0\wispr_v2.0\src\wispr.c \
 ..\atmel\wispr_v2.0\wispr_v2.0\src\epoch.c

OBJS = $(SRCS:.c=.o)

.c.o:
	$(CC) $(CFLAGS) $(INCS) -c $<  -o $@

all:
	make dat_dump_sdcard
	make wav_dump_sdcard
	
dat_dump_sdcard: dat_dump_sdcard.c $(OBJS)
	$(CC) dat_dump_sdcard.c -o dat_dump_sdcard $(CFLAGS) $(INCS) $(OBJS)

wav_dump_sdcard:  wav_dump_sdcard.c $(OBJS)
	$(CC) wav_dump_sdcard.c -o wav_dump_sdcard $(CFLAGS) $(INCS) $(OBJS)

clean:
	rm $(OBJS)
	rm dat_dump_sdcard
	rm wav_dump_sdcard


mingw_clean:
	del *.o
	del dat_dump_sdcard
	del wav_dump_sdcard
