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

/* the masks to read, = 0xf means surface */ 
typedef uint8_t nuphase_buffer_mask_t; 
const nuphase_buffer_mask_t NUPHASE_SURFACE_MASK = 0xff; 

typedef struct nuphase_trigger_enable
{
  uint8_t enable_beamforming : 1; 
  uint8_t enable_beam8  : 1;  
  uint8_t enable_beam4a  : 1;  
  uint8_t enable_beam4b  : 1;  
} nuphase_trigger_enable_t; 


typedef struct nuphase_trigger_output_config
{
  uint8_t enable : 1; 
  uint8_t polarity : 1; 
  uint8_t send_1Hz : 1; 
  uint8_t width; 
} nuphase_trigger_output_config_t ; 


typedef struct nuphase_ext_input_config 
{
  uint8_t use_as_trigger : 1; 
  uint8_t gate_enable : 1; 
  uint16_t gate_width; 
} nuphase_ext_input_config_t; 

typedef enum nuphase_which_board
{
  MASTER = 0, 
  SLAVE = 1
} nuphase_which_board_t; 


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
  NP_RESET_CALIBRATE,      //!< recalibrates ADC if necessary  
  NP_RESET_ALMOST_GLOBAL, //!< everything but register settings 
  NP_RESET_GLOBAL    //! everything 
} nuphase_reset_t; 



#define SURFACE_ANTENNA_MASK_VPOL_0 0x1
#define SURFACE_ANTENNA_MASK_VPOL_1 0x2
#define SURFACE_ANTENNA_MASK_VPOL_2 0x4 
#define SURFACE_ANTENNA_MASK_HPOL_0 0x8
#define SURFACE_ANTENNA_MASK_LPDA_0 0x10
#define SURFACE_ANTENNA_MASK_LPDA_2 0x20

//note that the pretrigger is defined elsewhere! 
typedef struct nuphase_surface_setup
{
  uint8_t vpp_threshold; 
  uint8_t coincident_window_length; //10.7ns * value
  uint8_t antenna_mask; //see above surface antenna masks
  uint8_t n_coincident_channels; //1-6, 0 interpretted as 1
  uint8_t require_h_greater_than_v; 
  uint8_t highpass_filter; 
} nuphase_surface_setup_t;


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
 *
 * The access to the SPI file descriptor is locked when opening, so only one
 * process can hold it. 
 *
 *
 * @param spi_master_device_name The master SPI device (likely something like /dev/spidev2.0) 
 * @param spi_slave_device_name The slave SPI device (likely something like /dev/spidev1.0) , or 0 for single board mode
 * @param power_gpio_number If positive, the GPIO that controls the board (and should be enabled at start) 
 * @param thread_safe  If non-zero a mutex will be initialized that will control concurrent access 
 *                     to this device from multiple threads.
 *
 *
 * @returns a pointer to the file descriptor, or 0 if something went wrong. 
 */
nuphase_dev_t * nuphase_open(const char * spi_master_device_name, 
                             const char * spi_slave_device_name, 
                             int power_gpio_number, 
                             int thread_safe) ; 

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
 * @param type The type of reset to do. See the documentation for nuphase_reset_t 
 * After reset, the phased trigger will be disabled and will need to be enabled if desired. 
 * @returns 0 on success
 */
int nuphase_reset(nuphase_dev_t *d, nuphase_reset_t type); 

/**Retrieve the board id for the current event. */
uint8_t nuphase_get_board_id(const nuphase_dev_t * d, nuphase_which_board_t which_board) ; 


/** Set the length of the readout buffer. Can be anything between 0 and 2048. (default is 624). */ 
void nuphase_set_buffer_length(nuphase_dev_t *d, uint16_t buffer); 

void nuphase_set_surface_buffer_length(nuphase_dev_t *d, uint16_t buffer); 

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
 * be non-zero unless interrupted or the timeout is reached or we also wait for surface triggers. 
 *
 * A timeout may be passed in seconds if you don't want to wait forever (and who wouldn't?) 
 *
 * If ths slave board exists, we will always wait on the slave board
 *
 * If surface_wait is non-zero, then nuphase_wait will also check surface buffers and return in that case (but non-surface buffers
 * are always checked in first). 
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
int nuphase_wait(nuphase_dev_t *d, nuphase_buffer_mask_t * ready, float timeout_seconds, int * surface_wait); 

/** Checks to see which buffers are ready to be read
 * If next_buffer is non-zero, will fill it with what the board things the next buffer to read is. 
 * if surface non-zero, will check surface as well; 
 * */ 
nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t *d, uint8_t*  next_buffer, nuphase_which_board_t which, int * surface);

/** Retrieve the firmware info */
int nuphase_fwinfo(nuphase_dev_t *d, nuphase_fwinfo_t* fwinfo, nuphase_which_board_t which); 



/** Fills in the status struct. 
 * if read_surface is 1, the surface scalers will also be read from the SLAVE board (normally scalers are read from the MASTER)
 **/ 
int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * stat, int read_surface); 

/**
 * Highest level read function. This will wait for data, read it into the 
 * required number of events, clear the buffer, and increment the event number appropriately 
 *
 * You must pass it a pointer to an array of the maximum number of buffers e.g. 
 
  \verbatim
  int nread; 
  nuphase_header_t headers[NP_NUM_BUFFER]; 
  nuphase_event_t events[NP_NUM_BUFFER]; 
  nread = wait_for_and_read_multiple_events(device, &headers, &events,0,0,0); 
  \endverbatim
 
 * OR the dynamically allocated variant
 
  \verbatim
  int nread; 
  nuphase_header_t (*headers)[NP_NUM_BUFFER] = malloc(sizeof(*headers)); 
  nuphase_header_t (*events)[NP_NUM_BUFFER] = malloc(sizeof(*headers)); 
  nread = wait_for_and_read_multiple_events(device, &headers, &events,0,0,0); 
 
  \endverbatim
 *
 *
 * May also wait for surface events if the surface pointers are non-zero and the surface trigger is enabled. 
 *
 * Returns the number of events read, excluding surface, which is passed to surface_read if non_zero . 
 *
 **/ 
int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                              nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                              nuphase_event_t  (*events)[NP_NUM_BUFFER], 
                                              nuphase_header_t * surface_header,
                                              nuphase_event_t  * surface_event, 
                                              int * surface_read
                                              ) ; 


/** Read a single event, filling header and event, and also clearing the buffer and increment event number. Does not check if there is anything available in the buffer.  
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
 * appropriately).
 *
 * Note that a mask value of SURFACE_MASK is special and will only read the surface array. 
 * Returns 0 on success. 
 *
 **/
int nuphase_read_multiple_array(nuphase_dev_t *d, nuphase_buffer_mask_t mask, 
                                nuphase_header_t *header_arr,  nuphase_event_t * event_arr
                                ); 
 
/** Reads buffers specified by mask. An pointer to event and header  must exist for each
 * buffer in the array pointed to by header_arr and event_arr ( Clears each
 * buffer after reading and increments event numbers appropriately).  Returns 0
 * on success. 
 *
 * Note that a mask value of SURFACE_MASK is special and will only read the surface array. 
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
 **/ 
int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t * result, nuphase_which_board_t which); 

/** Set the spi clock rate in MHz (default 10MHz)*/ 
int nuphase_set_spi_clock(nuphase_dev_t *d, unsigned clock); 

/** toggle chipselect between each transfer (Default yes) */ 
int nuphase_set_toggle_chipselect(nuphase_dev_t *d, int cs_toggle); 

/** toggle additional delay between transfers (Default 0) */ 
int nuphase_set_transaction_delay(nuphase_dev_t *d, unsigned delay_usecs); 



/** Set all the thresholds 
 * @param trigger_thresholds array of thresholds, should have NP_NUM_BEAMS members
 * @param dont_set_mask mask of beams not to set (pass 0 to set all). 
 */
int nuphase_set_thresholds(nuphase_dev_t* d, const uint32_t *trigger_thresholds, uint16_t dont_set_mask); 

/** Get all the thresholds
 * @param trigger_thresholds array of thresholds to be filled. Should have NP_NUM_BEAMS members.
 */
int nuphase_get_thresholds(nuphase_dev_t* d, uint32_t *trigger_thresolds); 

/** Set the trigger mask. 1 to use all beams */ 
int nuphase_set_trigger_mask(nuphase_dev_t* d, uint16_t mask); 

/* Get the trigger mask. */ 
uint16_t nuphase_get_trigger_mask(nuphase_dev_t* d); 


/** Set the attenuation for each channel .
 *  Should have NP_NUM_CHAN members for both master and slave. If 0, not applied for that board. 
 */ 
int nuphase_set_attenuation(nuphase_dev_t * d, const uint8_t * attenuation_master, const uint8_t * attenuation_slave); 

/** Get the attenuation for each channel .
 *  Should have NP_NUM_CHAN members. If 0, not read. 
 */ 
int nuphase_get_attenuation(nuphase_dev_t * d, uint8_t * attenuation_master, uint8_t  *attenuation_slave); 


/** Sets the channels used to form the trigger.
 * */ 
int nuphase_set_channel_mask(nuphase_dev_t * d, uint8_t channel_mask); 

/** Gets the channels used to form the trigger.
 * 8 LSB's for master, then for slave .
 * */ 
uint16_t nuphase_get_channel_mask(nuphase_dev_t *d); 

/** Sets the channel read mask */ 
int nuphase_set_channel_read_mask(nuphase_dev_t *d, nuphase_which_board_t board, uint8_t mask); 
int nuphase_set_surface_channel_read_mask(nuphase_dev_t *d, uint8_t mask); 

/** Gets the channel read mask */ 
uint8_t nuphase_get_channel_read_mask(nuphase_dev_t *d, nuphase_which_board_t board); 
uint8_t nuphase_get_surface_channel_read_mask(nuphase_dev_t *d); 


/** Set the trigger enables */ 
int nuphase_set_trigger_enables(nuphase_dev_t *d, nuphase_trigger_enable_t enable, nuphase_which_board_t which); 

/** Get the trigger enables */ 
nuphase_trigger_enable_t nuphase_get_trigger_enables(nuphase_dev_t *d, nuphase_which_board_t which); 

/** Enable or disable phased array readout. */ 
int nuphase_phased_trigger_readout(nuphase_dev_t *d, int enable);

/** Set the trigger holdoff (in the appropriate units) */ 
int nuphase_set_trigger_holdoff(nuphase_dev_t *d, uint16_t holdoff); 

/** Get the trigger holdoff */ 
uint16_t nuphase_get_trigger_holdoff(nuphase_dev_t *d); 

/** Set the pretrigger (0-7). Does it for both boards*/ 
int nuphase_set_pretrigger(nuphase_dev_t *d, uint8_t pretrigger, uint8_t surface_pretrigger); 

/** Get the pretrigger (0-7) (should be the same for both boards).
 *
 * This just returns the cached value, so it's possible that it was changed from underneath us. 
 * */ 
uint8_t nuphase_get_pretrigger(const nuphase_dev_t *d, uint8_t* surface_pretrigger); 

/** Set the external output config */ 
int nuphase_configure_trigger_output(nuphase_dev_t * d, nuphase_trigger_output_config_t config); 

/** get the external output config */ 
int nuphase_get_trigger_output(nuphase_dev_t * d, nuphase_trigger_output_config_t * config); 

/** Set the external triger config */ 
int nuphase_configure_ext_trigger_in(nuphase_dev_t * d, nuphase_ext_input_config_t config); 

/** get the external trigger config */ 
int nuphase_get_ext_trigger_in(nuphase_dev_t * d, nuphase_ext_input_config_t * config); 

int nuphase_enable_verification_mode(nuphase_dev_t * d, int mode); 

/* 0 if not on, 1 if on, -1 if error */ 
int nuphase_query_verification_mode(nuphase_dev_t * d); 

/** The poll interval for waiting, in us. If 0, will just do a sched_yield */ 
int nuphase_set_poll_interval(nuphase_dev_t *, unsigned short us); 

/** Sets the trigger delays. Should have NP_NUM_CHAN members */ 
int nuphase_set_trigger_delays(nuphase_dev_t *d, const uint8_t * delays); 

/** Gets the trigger delays. Should have NP_NUM_CHAN members */ 
int nuphase_get_trigger_delays(nuphase_dev_t *d, uint8_t * delays); 

/** Set the minimum threshold for any beam (default, 5000) */ 
int nuphase_set_min_threshold(nuphase_dev_t *d, uint32_t min_threshold); 


// this is necessary to toggle surface readout (otherwise nuphase_wait won't bother checking for surface events and nuphase*read won't actually do anything ) 
int nuphase_enable_surface_readout(nuphase_dev_t *d, int enable) ;


/** This turns off the surface channel ADC's. Must reset to clear */
int nuphase_surface_powerdown(nuphase_dev_t *d) ; 

/** Read a surface event, clearing the buffer. Does not check if anything is actually available! */ 
int nuphase_surface_read_event(nuphase_dev_t *d, nuphase_header_t * head,nuphase_event_t *ev); 


/** this throttles the number of times a surface event can be read in a second. 0 to clear. */ 
void nuphase_surface_set_throttle(nuphase_dev_t *d, int throttle, int clear_buffer_when_throttled); 

int nuphase_get_surface_skipped_in_last_second(nuphase_dev_t *d, int * last_second_skipped); 

// Configure the surface setup
int nuphase_configure_surface(nuphase_dev_t *d, const nuphase_surface_setup_t * s);


#endif
