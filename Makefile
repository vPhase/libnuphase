CC=gcc
LD=gcc
CFLAGS+=-fPIC -g


libnuphase.so: nuphasedaq.o 
	$(CC) -shared $< -o $@ 


clean: 
	rm -f *.o *.so 
