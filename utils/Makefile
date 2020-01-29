#
# Makefile 
#

CC=gcc

LIBS = 
INCS = -I..\src
CFLAGS = -W -O2 

SRCS = wav_file.c wispr.c epoch.c

OBJS = $(SRCS:.c=.o)

.c.o:
	$(CC) $(CFLAGS) $(INCS) -c $<  -o $@

update:
	cp ../src/wispr.c .
	cp ../src/epoch.c .
	
read_sdcard: read_sdcard.c $(OBJS)
	$(CC) -o read_sdcard read_sdcard.c $(OBJS) $(CFLAGS) $(INCS)

erase_sdcard:  $(OBJS)
	$(CC) -o erase_sdcard erase_sdcard.c $(OBJS) $(CFLAGS) $(INCS)

clean:
	rm $(OBJS)

mingw_read_sdcard: read_sdcard.c $(OBJS)
	$(CC) -o read_sdcard_mingw read_sdcard.c $(OBJS) $(CFLAGS) $(INCS)

mingw_clean:
	del $(OBJS)
