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

typedef enum nuphase_trigger_enable
{
  NP_TRIGGER_BEAMFORMING = 1, 
  NP_TRIGGER_EXTIN       = 2, 
  NP_TRIGGER_BEAM8       = 4, 
  NP_TRIGGER_BEAM4a       = 8, 
  NP_TRIGGER_BEAM4b       = 16, 
} nuphase_trigger_enable_t; 


typedef enum nuphase_which_board
{
  MASTER = 0, 
  SLAVE = 1
} nuphase_which_board_t; 


/** Configuration options, sent to fpga  with configure. 
 * Only values different from previously sent config will be sent
 * Use nuphase_config_init to fill with default values if you want. 
 */ 
typedef struct nuphase_config
{
 uint32_t trigger_thresholds[NP_NUM_BEAMS]; //!< The trigger thresholds  
 uint16_t trigger_mask;                     //!< Which triggers to use, default is all (0xffffff) 
 uint8_t  attenuation[NP_NUM_CHAN];         //!< per-channel attenuation
 uint8_t  channel_mask;                     //!< Which channels to use, default is all (0xff)
 nuphase_trigger_enable_t trigger_enables;           //!< Which trigger enables to use 
 uint8_t phased_trigger_readout;            //!< 1 to actually store events
 uint8_t trigger_holdoff;                   //!< Trigger holdoff
 uint8_t  pretrigger:3;                     //!< Amount of pre-trigger (multiple of 128 samples)  (3 bits);  
 nuphase_dev_t * master;                     
} nuphase_config_t; 

/** Fill the config with default options.*/ 
void nuphase_config_init(nuphase_config_t * c, nuphase_which_board_t which ); 


/** Firmware info retrieved from board */ 
typedef struct nuphase_fwinfo
{
  struct 
  {
    unsigned major : 4; 
    unsigned minor : 4; 
    unsigned master: 1; 
  } ver; 

  struct
  {
    unsigned year : 12; 
    unsigned month : 4; 
    unsigned day : 5; 
  } date;

  uint64_t dna;  //!< board dna 
} nuphase_fwinfo_t; 


typedef enum nuphase_reset_type 
{
  NP_RESET_COUNTERS, //!< resets event counter / trig number / trig time only 
  NP_RESET_ADC,      //!< resets and recalibrates ADC 
  NP_RESET_ALMOST_GLOBAL, //!< everything but register settings 
  NP_RESET_GLOBAL    //! everything 
} nuphase_reset_t; 



/** \brief Open a nuphase phased array board and initializes it. 
 *
 * This opens a nuphase phased array board and returns a pointer to the opaque
 * device handle. 
 *
 *
 * If that turns out to be too slow, I guess we can write a kernel driver. 
 *
 * Optionally, a mutex can be created to help synchronize access to this device
 * from multiple threads. 
 *
 * For now, the device handle also keeps track of the board id, buffer length
 * and the readout number offset. On initialization, the board id is set to the next
 * available id, buffer length is set to the default amount (624 samples) and
 * the readout number is set to unixtime << 32. They can be set to something
 * better using nuphase_set_board_id, nuphase_set_buffer_length and nuphase_set_readout_number_offset.
 *
 * The default nuphase_config_t is also sent on startup. Changes can be made
 * using nuphase_configure. 
 *
 * The access to the SPI file descriptor is locked when opening, so only one
 * process can hold it. 
 *
 *
 * @param spi_master_device_name The master SPI device (likely something like /dev/spidev2.0) 
 * @param spi_slave_device_name The slave SPI device (likely something like /dev/spidev1.0) , or 0 for single board mode
 * @param power_gpio_number If positive, the GPIO that controls the board (and should be enabled at start) 
 * @param cfg    If non-zero, this config is used instead of the default initial one.
 * @param cfg_slave    If non-zero, this config is used for the slave instead of the default initial one.
 * @param sync_access  If non-zero a mutex will be initialized that will control concurrent access 
 *                     to this device from multiple threads. ALSO ENABLES MASTER/SLAVE MODE ( enables synchronization of sw triggers / buffer clears) . 
 *
 *
 * @returns a pointer to the file descriptor, or 0 if something went wrong. 
 */
nuphase_dev_t * nuphase_open(const char * spi_master_device_name, 
                             const char * spi_slave_device_name, 
                             int power_gpio_number, 
                             const nuphase_config_t * cfg,
                             const nuphase_config_t * cfg_slave,
                             int sync_access) ; 

/** Deinitialize the phased array device and frees all memory. Do not attempt to use the device after closing. */ 
int nuphase_close(nuphase_dev_t * d); 

/**Set the board id for the device. Note that slave will be number +1. */
void nuphase_set_board_id(nuphase_dev_t * d, uint8_t number, nuphase_which_board_t which_board) ;


/** Set the readout number offset. Currently DOES NOT reset the counter
 * on the board (only done on nuphase_open / nuphase_reset). 
 *
 * This means you must run this either right after open or reset and before reading any buffers
 * for you to get something sensible.  
 * @param d board handle
 * @param offset the offset to set
 *
 **/ 
void nuphase_set_readout_number_offset(nuphase_dev_t * d, uint64_t offset); 


/** Sends a board reset. The reset type is specified by type. 
 *
 * @param d the board to reset
 * @param c the config to send right after resetting (unlike nuphase_open, this is not initialized to default if null and will result in a crash) . 
 * @param type The type of reset to do. See the documentation for nuphase_reset_t 
 * @returns 0 on success
 */
int nuphase_reset(nuphase_dev_t *d, const nuphase_config_t * c_master, const nuphase_config_t * c_slave, nuphase_reset_t type); 

/**Retrieve the board id for the current event. */
uint8_t nuphase_get_board_id(const nuphase_dev_t * d, nuphase_which_board_t which_board) ; 


/** Set the length of the readout buffer. Can be anything between 0 and 2048. (default is 624). */ 
void nuphase_set_buffer_length(nuphase_dev_t *d, uint16_t buffer); 

/** Retrieves the current buffer length */ 
uint16_t nuphase_get_buffer_length(const nuphase_dev_t *d); 


/** Send a software trigger to the device
 * @param d the device to send a trigger to. 
 *
 **/ 
int nuphase_sw_trigger(nuphase_dev_t * d); 

/** Change the state of the calpulser */ 
int nuphase_calpulse(nuphase_dev_t * d, unsigned state) ; 

/** Waits for data to be available, or time out, or nuphase_cancel_wait. 
 * 
 * Will busy poll nuphase_check_buffers (which) 
 *
 * If ready is passed, it will be filled after done waiting. Normally it should
 * be non-zero unless interrupted or the timeout is reached. 
 *
 * A timeout may be passed in seconds if you don't want to wait forever (and who wouldn't?) 
 *
 * The "correct way" to interrupt this by using nuphase_cancel_wait (either
 * from a signal handler or another thread). 
 *
 * If interrupted, (normally by nuphase_cancel_wait,), will return EINTR and ready (if passed) will be set to 0. 
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
int nuphase_wait(nuphase_dev_t *d, nuphase_buffer_mask_t * ready, float timeout_seconds, nuphase_which_board_t which); 

/** Checks to see which buffers are ready to be read
 * If next_buffer is non-zero, will fill it with what the board things the next buffer to read is. 
 * */ 
nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t *d, uint8_t*  next_buffer, nuphase_which_board_t which);

/** Retrieve the firmware info */
int nuphase_fwinfo(nuphase_dev_t *d, nuphase_fwinfo_t* fwinfo, nuphase_which_board_t which); 

/** Sends config to the board. The config is stuff like pretrigger threshold, 
 * Note that it's possible this may not take effect on the immediate next buffer. 
 *
 * @param force reconfigure even if matches current value by board (mostly useful internally) 
 * */ 
int nuphase_configure(nuphase_dev_t *d, const nuphase_config_t * config, int force, nuphase_which_board_t which); 



/** Fills in the status struct. 
 **/ 
int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * stat, nuphase_which_board_t which); 

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
                                              nuphase_header_t (*headers_master)[NP_NUM_BUFFER], 
                                              nuphase_event_t  (*events_master)[NP_NUM_BUFFER]
                                              
                                              ) ; 


/** Read a single event, filling header and event, and also clearing the buffer and increment event number. Does not check if there is anyting available in the buffer.  
 *
 * @param d the device handle
 * @param buffer the buffer to read
 * @param header header to write to 
 * @param event event to write to 
 * Returns 0 on success.
 * */ 
int nuphase_read_single(nuphase_dev_t *d, uint8_t buffer, 
                        nuphase_header_t * header, nuphase_event_t * event
                        );


/** Reads buffers specified by mask. An event and header  must exist for each
 * buffer in the array pointed to by header_arr and event_arr ( Clears each buffer after reading and increments event numbers
 * appropriately).  Returns 0 on success. 
 *
 **/
int nuphase_read_multiple_array(nuphase_dev_t *d, nuphase_buffer_mask_t mask, 
                                nuphase_header_t *header_arr,  nuphase_event_t * event_arr
                                ); 
 
/** Reads buffers specified by mask. An pointer to event and header  must exist for each
 * buffer in the array pointed to by header_arr and event_arr ( Clears each
 * buffer after reading and increments event numbers appropriately).  Returns 0
 * on success. 
 **/
int nuphase_read_multiple_ptr(nuphase_dev_t *d, nuphase_buffer_mask_t mask, 
                              nuphase_header_t **header_ptr_arr,  nuphase_event_t ** event_ptr_arr
                              ); 



/** Lowest-level waveform read command. 
 * Read the given addresses from the buffer and channel and put into data (which should be the right size). 
 * Does not clear the buffer or increment event number. 
 *
 **/ 
int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start_ram, uint8_t end_ram, uint8_t * data, nuphase_which_board_t which); 


/** Lowest-level write command. Writes 4 bytes from buffer to device (if master/slave, to both) */ 
int nuphase_write(nuphase_dev_t *d, const uint8_t* buffer); 

/** Lowest-level read command. Reads 4 bytes into buffer */ 
int nuphase_read(nuphase_dev_t *d, uint8_t* buffer, nuphase_which_board_t which); 

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
 * @param slave   if 1, read from slave instead of master (or single board) 
 * @return 0 on success
 * 
 * 
 **/ 
int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t * result, nuphase_which_board_t which); 


//TODO: 
/** Set the spi clock rate in MHz (default 10MHz)*/ 
int nuphase_set_spi_clock(nuphase_dev_t *d, unsigned clock); 

/** toggle chipselect between each transfer (Default yes) */ 
int nuphase_set_toggle_chipselect(nuphase_dev_t *d, int cs_toggle); 

/** toggle additional delay between transfers (Default 0) */ 
int nuphase_set_transaction_delay(nuphase_dev_t *d, unsigned delay_usecs); 



#endif
