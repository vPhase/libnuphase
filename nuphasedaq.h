#ifndef _nuphasedaq_h
#define _nuphasedaq_h

#include "nuphase.h" 


#define NP_NUM_CHUNK 4 
#define NP_WORD_SIZE 4 



/** opaque handle for device */ 
struct nuphase_dev; 
/** typedef for device */ 
typedef struct nuphase_dev nuphase_dev_t; 

/** a bitmask indicating which buffers are available */ 
typedef uint8_t nuphase_buffer_mask_t;


/** \brief Open a nuphase phased array board and initializes it. 
 *
 * This opens a nuphase phased array board and returns a pointer to the opaque
 * device handle. 
 *
 * Currently, communication with the board is via userspace SPI (spidev) and
 * optionally, a userspace interrupt (using uio_pdrv_genirq, as in
 * https://yurovsky.github.io/2014/10/10/linux-uio-gpio-interrupt/) to know
 * when data is available without polling. 
 *
 * Optionally, a mutex can be created to help synchornize access to this device from multiple threads. 
 *
 * For now, the device handle also keeps track of the buffer length and the event number. On initialization, the
 * buffer length is set to the default amount (624 samples) and the event number is set to unixtime << 32. They can be
 * set to something better using nuphase_set_buffer_length and nuphase_set_event_number.
 *
 * The dfeault nuphase_config_t is also sent on startup. Changes can be made using nuphase_configure. 
 *
 * If that turns out to be too slow, I guess we can write a kernel driver. 
 *
 * @param spi_device_name The SPI device (likely something like  /sys/bus/spi/devices/spi1.0 )
 * @param gpio_interrupt_device_name The GPIO device acting as an interrupt (e.g. /dev/uio0). This is purely optional, if not defined, we will busy wait. 
 * @param lock_access  If 1, a mutex will be initialized that will control concurrent access to this device from multiple threads
 * 
 */

nuphase_dev_t * nuphase_open(const char * spi_device_name, const char *
    gpio_interrupt_device_name, int lock_access); 


/** Deinitialize the phased array device and frees all memory. Do not attempt to use the device after closing. */ 
int nuphase_close(nuphase_dev_t * d); 


/**Set the event number for the device */
void nuphase_set_event_number(nuphase_dev_t * d, uint64_t number) ;

/**Retrieve the event number for the current event */
uint64_t nuphase_get_event_number(const nuphase_dev_t * d) ; 

/** Set the length of the readout buffer. Can be anything between 0 and 2048. (default is 624). */ 
void nuphase_set_buffer_length(nuphase_dev_t *d, uint16_t buffer); 

/** Retrieves the current buffer length */ 
uint16_t nuphase_get_buffer_length(const nuphase_dev_t *d); 


/** Send a software trigger to the device */ 
int nuphase_sw_trigger(nuphase_dev_t * d); 

/** Change the state of the calpulser */ 
int nuphase_calpulse(nuphase_dev_t * d, unsigned state) ; 

/** Waits for data to be available (or might be interrupted by a signal). Returns 
 * the buffer mask at the end (if it's 0, it means nothing is ready to read */
nuphase_buffer_mask_t nuphase_wait(nuphase_dev_t *d); 

/** Checks to see which buffers are ready to be read */ 
nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t *d);

/** Retrieve the firmware info */
int nuphase_fwinfo(nuphase_dev_t *d, nuphase_fwinfo_t* fwinfo); 

/** Sends config to the board*/ 
int nuphase_configure(nuphase_dev_t *d, const nuphase_config_t * config); 


/* Highest level read function. This will wait for data, read it into the 
 * required number of events, clear the buffer, and increment the event number appropriately 
 *
 * You must pass it a pointer to an array of the maximum number of buffers e.g. 
 
  \verbatim
  int nread; 
  nuphase_header_t headers[NP_NUM_BUFFER]; 
  nuphase_event_t events[NP_NUM_BUFFER]; 
  nread = wait_for_and_read_multiple_events(device, &headers, &events); 
  \endverbatim
 
 * OR the dynamically allocated variant
 
  \verbatim
  int nread; 
  nuphase_header_t (*headers)[NP_NUM_BUFFER] = malloc(sizeof(*headers)); 
  nuphase_header_t (*events)[NP_NUM_BUFFER] = malloc(sizeof(*headers)); 
  nread = wait_for_and_read_multiple_events(device, &headers, &events); 
 
  \endverbatim
 *
 *
 * Returns the number of events read. 
 *
 * */ 
int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                              nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                              nuphase_event_t  (*events)[NP_NUM_BUFFER]) ; 


/** Read a single event, filling header and event, and also clearing the buffer and increment event number. Does not check if there is anyting available in the buffer.  
 * Returns 0 on success.
 * */ 
int nuphase_read_single(nuphase_dev_t *d, uint8_t buffer, 
                        nuphase_header_t * header, nuphase_event_t * event);


/** Reads buffers specified by mask. An event and header  must exist for each mask. ( Clears each buffer after reading and increments event numbers appropriately.
 * Returns 0 on success. 
 **/
int nuphase_read_multiple(nuphase_dev_t *d, nuphase_buffer_mask_t mask, nuphase_header_t *header_arr,  nuphase_event_t * event_arr); 



/** Lowest-level waveform read command. 
 * Read the given addresses from the buffer and channel and put into data (which should be the right size). 
 * Does not clear the buffer or increment event number. 
 *
 **/ 
int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start_ram, uint8_t end_ram, uint8_t * data); 


/** Clear the specified buffer. Returns 0 on success. */ 
int nuphase_clear_buffer(nuphase_dev_t *d, uint8_t buffer); 


/** 
 * low level register read 
 *
 * @param d device handle
 * @param address The register address to read
 * @param result  where to store the result, should be 4 bytes. 
 * @return 0 on success
 * 
 * 
 **/ 
int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t * result); 



#endif
