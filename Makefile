CC=gcc
LD=gcc
CFLAGS+=-fPIC -g -Wall
LDFLAGS+= -lpthread -lz 

PREFIX=/usr/local/ 
LIBDIR=lib 

.PHONY: clean install doc

HEADERS = nuphase.h nuphasedaq.h 
OBJS = nuphase.o nuphasedaq.o

libnuphase.so: $(OBJS) $(HEADERS)
	$(CC) -shared $(OBJS) -o $@ $(LDFLAGS) 

install:  doc
	install -d $(PREFIX)/$(LIBDIR)
	install -d $(PREFIX)/$(INCLUDE) 
	install -d $(PREFIX)/$(SHARE) 
	install libnuphase.so $(PREFIX)/$(LIBDIR)  
	install $(HEADERS) $(PREFIX)/include 
	install doc $(PREFIX)/share 


doc: 
	doxygen doc/Doxyfile 

clean: 
	rm -f *.o *.so 
	rm -rf doc/latex
	rm -rf doc/html
	rm -rf doc/man
