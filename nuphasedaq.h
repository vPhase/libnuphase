#ifndef _nuphasedaq_h
#define _nuphasedaq_h

#include "nuphase.h" 






/* opaque handle for device */ 
struct nuphase_dev; 
typedef struct nuphase_dev nuphase_dev_t; 
typedef uint32_t nuphase_buffer_mask_t;


nuphase_dev_t * nuphase_open(const char * spi_device_name, 
                             const char * gpio_interrupt_device_name); 


int nuphase_close(nuphase_dev_t * d); 


/* low level register read 
 * result should be 4 bytes 
 **/ 
int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t * result); 

int nuphase_sw_trigger(nuphase_dev_t * d, unsigned state); 

int nuphase_calpulse(nuphase_dev_t * d, unsigned state) ; 

/* Waits for data to be available (or might be interrupted by a signal) 
 **/ 
int nuphase_wait(nuphase_dev_t *d); 

/* Checks if data is ready. How?  */ 
//nuphase_buffer_mask_t nuphase_check_ready(nuphase_dev_t *d);

/* Low-level read command. */ 
int nuphase_read_raw(nuphase_dev_t *d, uint8_t channel, uint8_t start_ram, uint8_t end_ram, uint8_t * data); 

int nuphase_version(nuphase_dev_t *d, nuphase_version_t* version); 

//int nuphase_configure(nuphase_dev_t *d, const nuphase_config_t * config); 
//int nuphase_status(nuphase_dev_t *d, nuphase_status_t * status); 
//int nuphase_read_single(nuphase_dev_t *d, unsigned buffer_number, nuphase_event_t * event);
/* Reads buffers specified by mask. An event must exist for each mask  */
//int nuphase_read_multiple(nuphase_dev_t *d, nuphase_buffer_mask_t mask, nuphase_event_t ** events); 









#endif
