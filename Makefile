CC=gcc
LD=gcc
CFLAGS+=-fPIC -g
LDFLAGS+= -lpthread

libnuphase.so: nuphasedaq.o 
	$(CC) -shared $< -o $@ $(LDFLAGS) 

clean: 
	rm -f *.o *.so 
