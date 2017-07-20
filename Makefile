CC=gcc
LD=gcc
CFLAGS+=-fPIC -g -Wall -Wextra -O2
LDFLAGS+= -lpthread -lz 

#uncomment to enable excessive printouts
#CFLAGS+=-DDEBUG_PRINTOUTS

PREFIX=/usr/local/ 
LIBDIR=lib 

.PHONY: clean install doc

HEADERS = nuphase.h nuphasedaq.h 
OBJS = nuphase.o nuphasedaq.o

libnuphase.so: $(OBJS) $(HEADERS)
	$(CC) -shared $(OBJS) -o $@ $(LDFLAGS) 

install-doc:
	install -d $(PREFIX)/$(SHARE) 
	install doc $(PREFIX)/share 

install:  
	install -d $(PREFIX)/$(LIBDIR)
	install -d $(PREFIX)/$(INCLUDE) 
	install libnuphase.so $(PREFIX)/$(LIBDIR)  
	install $(HEADERS) $(PREFIX)/include 

doc: 
	doxygen doc/Doxyfile 

nuphase.pdf: doc 
	make -C doc/latex  && cp doc/latex/refman.pdf $@ 

clean: 
	rm -f *.o *.so 
	rm -rf doc/latex
	rm -rf doc/html
	rm -rf doc/man
