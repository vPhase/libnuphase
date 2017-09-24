### Some settings one might reasonably want to change

#this is only needed on the DAQ really 
ENABLE_CURL=1

# if set, will by default use /tmp/intercepttty insead of /dev/ttyUSB0 as the serial device
SERIAL_DEBUG=0

# set  this if you want to see SPI transactions in gory detail 
SPI_DEBUG=0


CC=gcc
LD=gcc

#I'm lazy and using implicit rules for now, which means everything gets the same cflags
CFLAGS+=-fPIC -g -Wall -Wextra  -D_GNU_SOURCE
LDFLAGS+= -lz -g

DAQ_LDFLAGS+= -lpthread -L./ -lnuphase -g 


ifeq ($(ENABLE_CURL),1) 
	CFLAGS+=-DWITH_CURL
	DAQ_LDFLAGS+= `curl-config --libs`
endif

ifeq ($(SPI_DEBUG),1)
	CFLAGS+=-DDEBUG_PRINTOUTS
endif

ifeq ($(SERIAL_DEBUG),1)
	CFLAGS+=-DDEBUG_SERIAL 
endif


PREFIX=/usr/local/ 
LIBDIR=lib 

.PHONY: clean install doc install-doc all client



HEADERS = nuphase.h 
OBJS = nuphase.o 

DAQ_HEADERS = nuphasedaq.h nuphasehk.h bbb_gpio.h bbb_ain.h 
DAQ_OBJS =  bbb_gpio.o bbb_ain.o nuphasehk.o nuphasedaq.o 

all: libnuphase.so libnuphasedaq.so 

client: libnuphase.so 

libnuphase.so: $(OBJS) $(HEADERS)
	$(CC) $(LDFLAGS)  -shared $(OBJS) -o $@

libnuphasedaq.so: $(DAQ_OBJS) $(DAQ_HEADERS) libnuphase.so 
	$(CC) $(LDFLAGS) $(DAQ_LDFLAGS) -shared $(DAQ_OBJS) -o $@ 

install-doc:
	install -d $(PREFIX)/$(SHARE) 
	install doc $(PREFIX)/share 

install-client:  client 
	install -d $(PREFIX)/$(LIBDIR)
	install -d $(PREFIX)/$(INCLUDE) 
	install libnuphase.so $(PREFIX)/$(LIBDIR)  
	install $(HEADERS) $(PREFIX)/include 
	
install:  all install-client 
	install $(DAQ_HEADERS) $(PREFIX)/include 
	install libnuphasedaq.so $(PREFIX)/$(LIBDIR)  

doc: 
	doxygen doc/Doxyfile 

nuphase.pdf: doc 
	make -C doc/latex  && cp doc/latex/refman.pdf $@ 

clean: 
	rm -f *.o *.so 
	rm -rf doc/latex
	rm -rf doc/html
	rm -rf doc/man
