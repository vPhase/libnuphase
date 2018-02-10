CC=gcc 
CFLAGS+=-g  -I../ -Wall -Wextra -D_GNU_SOURCE
LDFLAGS+= -L../ -lnuphasedaq -lnuphase -lpthread -lm -lz


EXAMPLES=get_info read_word write_word read_status read_event register_test \
				 global_reset io read_register dump_events dump_headers read_many \
				 check_for_glitches watch_register double_read reset_adcs attenuation_scan \
				 read_ain read_hk turn_on_boards turn_off_boards read_asps_heater set_asps_heater dump_hk dump_status dump_shared_hk

all: $(EXAMPLES) 

clean: 
	rm -f $(EXAMPLES) 



