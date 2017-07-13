CC=gcc
LD=gcc
CFLAGS+=-fPIC -g -Wall
LDFLAGS+= -lpthread -lz 

libnuphase.so: nuphase.o nuphasedaq.o 
	$(CC) -shared $^ -o $@ $(LDFLAGS) 

clean: 
	rm -f *.o *.so 
