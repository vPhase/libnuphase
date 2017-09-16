CC=gcc
LD=gcc
CFLAGS+=-fPIC -g -Wall -Wextra 
LDFLAGS+= -lz 
DAQ_LDFLAGS+= -lpthread -lcurl  

#uncomment to enable excessive printouts
#CFLAGS+=-DDEBUG_PRINTOUTS

PREFIX=/usr/local/ 
LIBDIR=lib 

.PHONY: clean install doc install-doc all client



HEADERS = nuphase.h 
OBJS = nuphase.o 

DAQ_HEADERS = nuphasedaq.h nuphasehk.h bbb_gpio.h bbb_ain.h 
DAQ_OBJS = nuphasedaq.o nuphase.o bbb_gpio.o bbb_ain.o 

all: libnuphase.so libnuphasedaq.so 

client: libnuphase.so 


libnuphase.so: $(OBJS) $(HEADERS)
	$(CC) -shared $(OBJS) -o $@ $(LDFLAGS) 

libnuphasedaq.so: $(DAQ_OBJS) $(DAQ_HEADERS)
	$(CC) -shared $(DAQ_OBJS) -o $@ $(DAQ_LDFLAGS) 


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
