#ifndef _nuphasedaq_h
#define _nuphasedaq_h

#include "nuphase.h" 

/** \file nuphasedaq.h  
 *
 * Include file for talking to the hardware. 
 *
 * Cosmin Deaconu <cozzyd@kicp.uhicago.edu> 
 *
 * This header defines structures, constants, and functions
 * for working with the phased array hardware. 
 *
 * There are 
 * 
 *
 */


/** Number of chunks in an address */ 
#define NP_NUM_CHUNK 4 

/** Number of bytes in a word */
#define NP_WORD_SIZE 4 



/** opaque handle for device */ 
struct nuphase_dev; 
/** typedef for device */ 
typedef struct nuphase_dev nuphase_dev_t; 

/** a bitmask indicating which buffers are available */ 
typedef uint8_t nuphase_buffer_mask_t;

/** Configuration options, sent to fpga  with configure. 
 * Only values different from previously sent config will be sent
 * Use nuphase_config_init to fill with default values if you want. 
 */ 
typedef struct nuphase_config
{
 uint32_t trigger_thresholds[NP_NUM_BEAMS]; //!< The trigger thresholds  
 uint16_t trigger_mask;                     //!< Which triggers to use, default is all (0xffffff) 
 uint8_t  channel_mask;                     //!< Which channels to use, default is all (0xff)
 uint8_t  pretrigger:3;                     //!< Amount of pre-trigger (multiple of 85 samples)  (3 bits);  
} nuphase_config_t; 

/** Fill the config with default options */
void nuphase_config_init(nuphase_config_t * c); 


/** Firmware info retrieved from board */ 
typedef struct nuphase_fwinfo
{
  uint32_t ver;  //!< firmware version
  uint32_t date; //!< firmware date
  uint64_t dna;  //!< board dna 
} nuphase_fwinfo_t; 


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
 * Optionally, a mutex can be created to help synchornize access to this device
 * from multiple threads. 
 *
 * For now, the device handle also keeps track of the board id, buffer length
 * and the event number. On initialization, the board id is set to the next
 * available id, buffer length is set to the default amount (624 samples) and
 * the event number is set to unixtime << 32. They can be set to something
 * better using nuphase_set_board_id, nuphase_set_buffer_length and nuphase_set_event_number.
 *
 * The default nuphase_config_t is also sent on startup. Changes can be made
 * using nuphase_configure. 
 *
 * The access to the SPI file descriptor is locked when opening, so only one
 * process can hold it. 
 *
 * If that turns out to be too slow, I guess we can write a kernel driver. 
 *
 * @param spi_device_name The SPI device (likely something like
 * /sys/bus/spi/devices/spi1.0 ) @param gpio_interrupt_device_name The GPIO
 * device acting as an interrupt (e.g. /dev/uio0). This is purely optional, if
 * not defined, we will busy wait.  @param cfg          If non-zero, this
 * config is used instead of the default initial one.  @param lock_access  If
 * 1, a mutex will be initialized that will control concurrent access to this
 * device from multiple threads @returns a pointer to the file descriptor, or 0
 * if something went wrong. 
 */
nuphase_dev_t * nuphase_open(const char * spi_device_name, const char *
    gpio_interrupt_device_name, const nuphase_config_t * cfg, int lock_access); 

/** Deinitialize the phased array device and frees all memory. Do not attempt to use the device after closing. */ 
int nuphase_close(nuphase_dev_t * d); 

/**Set the event number for the device */
void nuphase_set_event_number(nuphase_dev_t * d, uint64_t number) ;

/**Retrieve the event number for the current event */
uint64_t nuphase_get_event_number(const nuphase_dev_t * d) ; 

/**Set the board id for the device */
void nuphase_set_board_id(nuphase_dev_t * d, uint8_t number) ;

/**Retrieve the board id for the current event */
uint8_t nuphase_get_board_id(const nuphase_dev_t * d) ; 



/** Set the length of the readout buffer. Can be anything between 0 and 2048. (default is 624). */ 
void nuphase_set_buffer_length(nuphase_dev_t *d, uint16_t buffer); 

/** Retrieves the current buffer length */ 
uint16_t nuphase_get_buffer_length(const nuphase_dev_t *d); 


/** Send a software trigger to the device */ 
int nuphase_sw_trigger(nuphase_dev_t * d); 

/** Change the state of the calpulser */ 
int nuphase_calpulse(nuphase_dev_t * d, unsigned state) ; 

/** Waits for data to be available, or time out, or nuphase_cancel_wait. 
 * 
 * If a hardware interrupt is available, we will wait for that. Otherwise we
 * will keep polling nuphase_check_buffers. 
 *
 * If ready is passed, it will be filled after done waiting. Normally it should
 * be non-zero unless interrupted or the timeout is reached. 
 *
 * A timeout may be passed in seconds if you don't want to wait forever (and who wouldn't?) 
 *
 * The "correct way" to interrupt this by using nuphase_cancel_wait (either
 * from a signal handler or another thread). 
 *
 * If interrupted, (normally by nuphase_cancel_wait, although when using the
 * GPIO interrupt, could also be interrupted by a signal), will return EINTR and ready (if passed) will be set to 0. 
 *
 * We also immediately return EAGAIN if there is a previous call to nuphase_cancel_wait that didn't actually cancel anything (like
 * if it was called when nothing was waiting). 
 *
 * Right now only one thread is allowed to wait at a time. If you try waiting from another
 * thread, it will return EBUSY. This is only enforced if the device has locks enabled.  
 *
 * Returns 0 on success,  
 * 
 **/
int nuphase_wait(nuphase_dev_t *d, nuphase_buffer_mask_t * ready, float timeout_seconds); 

/** Checks to see which buffers are ready to be read */ 
nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t *d);

/** Retrieve the firmware info */
int nuphase_fwinfo(nuphase_dev_t *d, nuphase_fwinfo_t* fwinfo); 

/** Sends config to the board. The config is stuff like pretrigger threshold, 
 * Note that it's possible this may not take effect on the immediate next buffer. 
 * */ 
int nuphase_configure(nuphase_dev_t *d, const nuphase_config_t * config); 



/** Fills in the status struct 
 **/ 
int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * stat); 

/**
 * Highest level read function. This will wait for data, read it into the 
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
 **/ 
int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                              nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                              nuphase_event_t  (*events)[NP_NUM_BUFFER]) ; 


/** Read a single event, filling header and event, and also clearing the buffer and increment event number. Does not check if there is anyting available in the buffer.  
 *
 * @param d the device handle
 * @param buffer the buffer to read
 * @param header header to write to 
 * @param event event to write to 
 * Returns 0 on success.
 * */ 
int nuphase_read_single(nuphase_dev_t *d, uint8_t buffer, 
                        nuphase_header_t * header, nuphase_event_t * event);


/** Reads buffers specified by mask. An event and header  must exist for each
 * buffer in the array pointed to by header_arr and event_arr ( Clears each buffer after reading and increments event numbers
 * appropriately).  Returns 0 on success. 
 *
 **/
int nuphase_read_multiple_array(nuphase_dev_t *d, nuphase_buffer_mask_t mask, 
                                nuphase_header_t *header_arr,  nuphase_event_t * event_arr); 
 
/** Reads buffers specified by mask. An pointer to event and header  must exist for each
 * buffer in the array pointed to by header_arr and event_arr ( Clears each
 * buffer after reading and increments event numbers appropriately).  Returns 0
 * on success. 
 **/
int nuphase_read_multiple_ptr(nuphase_dev_t *d, nuphase_buffer_mask_t mask, 
                              nuphase_header_t **header_ptr_arr,  nuphase_event_t ** event_ptr_arr); 



/** Lowest-level waveform read command. 
 * Read the given addresses from the buffer and channel and put into data (which should be the right size). 
 * Does not clear the buffer or increment event number. 
 *
 **/ 
int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start_ram, uint8_t end_ram, uint8_t * data); 


/** Lowest-level write command. Writes 4 bytes from buffer */ 
int nuphase_write(nuphase_dev_t *d, const uint8_t* buffer); 

/** Lowest-level read command. Reads 4 bytes into buffer */ 
int nuphase_read(nuphase_dev_t *d, uint8_t* buffer); 

/** Clear the specified buffers. Returns 0 on success. */ 
int nuphase_clear_buffer(nuphase_dev_t *d, nuphase_buffer_mask_t mask); 

/** This cancels the current nuphase_wait. If there
 * is no nuphase_wait, it will prevent the first  future one from running
 * Should be safe to call this from a signal handler (hopefully :). 
 */
void nuphase_cancel_wait(nuphase_dev_t *d) ; 

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
