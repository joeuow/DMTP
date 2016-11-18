# Make file for this socket program model. 
VER=2
ifneq ($(VER), )
MVER:=0
EXTRAFLAGS:=-DBUILD_VERSION=$(MVER)$(VER)
else
EXTRAFLAGS:=-DBUILD_VERSION=1
endif

OBJ := accting.o threads.o watchdog.o checksum.o geozone.o io.o odometer.o \
base64.o comport.o os.o upload.o bintools.o event.o gpstools.o gpsmods.o \
packet.o random.o strtools.o utctools.o propman.o sockets.o pqueue.o socket.o \
buffer.o events.o gps.o log.o motion.o transport.o rfid.o protocol.o mainloop.o startup.o ap_diagnostic_log.o float_point_handle.o

SRC := $(OBJ:%.o=%.c)

CC=arm-linux-gcc
STRIP=arm-linux-strip -s
CFLAGS:=-Wall -O3
LIBS:=-lpthread -lm -lrt -lwlcomm -lssl -lcrypto -liwlist -liwconfig
#LPATH:=-L/opt/root-filesystem/usr/lib
LPATH:=-L/opt/philips/root-filesystem/usr/lib
INCLUDES:=-I/opt/old-tools/host/usr/include -I/opt/old-tools/host/usr/arm-linux/include -I/opt/old-tools/host/usr/arm-linux/sysroot/usr/include
CPPFLAGS:=-DTRANSPORT_MEDIA_SOCKET -DPROTOCOL_THREAD -DGPS_THREAD -DDEBUG_COMPILE -DTARGET_LINUX
ifeq ($(shell date +%Z),EDT)
    ZONE:=1
else
    ZONE:=0
endif
EXTRAFLAGS+=-DTIMEZONE=$(ZONE)

all:	ositechdmtp

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) $(CPPFLAGS) -o $@ -c $<
#	$(CC) $(CFLAGS) $(INCLUDES) $(CPPFLAGS) -c $<

startup.o: startup.c
	$(CC) $(CFLAGS) $(INCLUDES) $(CPPFLAGS) $(EXTRAFLAGS) -o $@ -c $<

ositechdmtp: $(OBJ)
	$(CC) $(CFLAGS) $(LPATH) $(LIBS) -o $@ $^
	$(STRIP) $@
#	sudo cp $@ /opt/root-filesystem/ositech/
	sudo cp $@ /opt/philips/root-filesystem/ositech/


.PHONY:	clean
clean:
	if [ -e "accting.o" ]; then rm *.o; fi
	if [ -e "ositechdmtp" ]; then rm ositechdmtp; fi

.PHONY:	clean_build
clean_build: clean ositechdmtp 


