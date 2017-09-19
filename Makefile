CC=gcc
LD=gcc
CFLAGS+=-fPIC -g -Wall -Wextra  -D_GNU_SOURCE
LDFLAGS+= -fPIC -lz -g

ENABLE_CURL=0

DAQ_LDFLAGS+= -lpthread -L./ -lnuphase
ifeq (ENABLE_CURL,1) 
	CFLAGS+=-DWITH_CURL 
	DAQ_LDFLAGS+= `curl-config --libs`
endif

#uncomment to enable excessive printouts
#CFLAGS+=-DDEBUG_PRINTOUTS

PREFIX=/usr/local/ 
LIBDIR=lib 

.PHONY: clean install doc install-doc all client



HEADERS = nuphase.h 
OBJS = nuphase.o 

DAQ_HEADERS = nuphasedaq.h nuphasehk.h bbb_gpio.h bbb_ain.h 
DAQ_OBJS = nuphasedaq.o nuphasehk.o bbb_gpio.o bbb_ain.o 

all: libnuphase.so libnuphasedaq.so 

client: libnuphase.so 

libnuphase.so: $(OBJS) $(HEADERS)
	$(CC) $(LDFLAGS)  -shared $(OBJS) -o $@

libnuphasedaq.so: $(DAQ_OBJS) $(DAQ_HEADERS)
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
