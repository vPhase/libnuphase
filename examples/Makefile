CC=gcc 
CFLAGS+=-g  -I../ -Wall
LDFLAGS+= -L../ -lnuphase -lm -lpthread


EXAMPLES=read_addr get_info read_word write_word read_status read_event register_test global_reset io read_register dump_events dump_headers read_many check_for_glitches watch_register read_many_from_multiple

all: $(EXAMPLES) 

clean: 
	rm -f $(EXAMPLES) 



