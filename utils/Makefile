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

read_sdcard:  $(OBJS)
	$(CC) -o read_sdcard read_sdcard.c $(OBJS) $(CFLAGS) $(INCS)

erase_sdcard:  $(OBJS)
	$(CC) -o erase_sdcard erase_sdcard.c $(OBJS) $(CFLAGS) $(INCS)

clean:
	rm $(OBJS)
	rm read_sdcard
	rm erase_sdcard

mingw_read_sdcard:  $(OBJS)
	$(CC) -o read_sdcard $(OBJS)

mingw_clean:
	del *.o
	del read_sdcard
