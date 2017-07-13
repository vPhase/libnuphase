#ifndef _nuphase_h
#define _nuphase_h

/** \file nuphase.h 
 *
 * Include file for working with nuphase data. 
 
 * Cosmin Deaconu <cozzyd@kicp.uchicago.edu> 
 *
 *
 * This header defines structures, constants and some utility functions for
 * nuphase.
 * 
 * Note that these are in-memory structs. The on-disk format does differ.  For
 * structs that are meant to be persisted (headers and events), use the
 * nuphase_X_write/read() functions. 
 * 
 * In particular, you should use the utility functions here to initailize and
 * read binary nuphase data from disk as it handles the versioning properly. 
 *
 */

#include <time.h>
#include <stdint.h>
#include <stdio.h> 
#include <zlib.h> 

/** The number of channels per board */
#define NP_NUM_CHAN 8 

/** The number of buffers */
#define NP_NUM_BUFFER 4 

/** The maximum length of a waveform */ 
#define NP_MAX_WAVEFORM_LENGTH 2048  

/** The number of trigger beams available */ 
#define NP_NUM_BEAMS 16 

/** Error codes for read/write */ 
typedef enum 
{
NP_ERR_CHECKSUM_FAILED  = 0xbadadd,  //!< checksum failed while reading
NP_ERR_NOT_ENOUGH_BYTES = 0xbadf00d, //!< did not write or read enough bytes
NP_ERR_WRONG_TYPE       = 0xc0fefe , //!< got nonsensical type
NP_ERR_BAD_VERSION      = 0xbadbeef  //!< version number not understood
} np_io_error_t; 


 
/**  Trigger types */ 
typedef enum nuphase_trigger_type 
{
  NP_TRIG_RF,    //!< triggered by input wavecforms
  NP_TRIG_SW,    //!< triggered by software (force trigger)  
  NP_TRIG_EXT    //!< triggered by external trigger 
} nuphase_trig_type_t; 

/** in memory layout of nuphase event headers. 
 *
 * STILL PRELIMINARY 
 *
 * On-disk layout is different and opaque, you must use nuphase_header_read() / nuphase_header_write() to write
 * to disk./ 
 *
 */
typedef struct nuphase_header 
{
  uint64_t event_number;               //!< the event number assigned to this event. Will match the event body. 
  uint16_t buffer_length;              //!< the buffer length. Stored both here and in the event. 
  uint16_t pretrigger_samples;         //!< Number of samples that are pretrigger
  uint32_t readout_time;               //!< CPU time of readout, seconds
  uint32_t readout_time_ns;            //!< CPU time of readout, nanoseconds 
  uint64_t trig_time;                  //!< Board trigger time
  uint16_t triggered_beams;            //!< The beams that triggered 
  uint16_t beam_mask;                  //!< The enabled beams
  uint32_t beam_power[NP_NUM_CHAN];    //!< The power in each beam at the trigger time
  nuphase_trig_type_t trig_type;       //!< The trigger type? 
  uint16_t deadtime;                   //!< ??? Will we have this available? If so, this will be a fraction
  uint8_t buffer_number;               //!< the buffer number (do we need this?) 
  uint8_t channel_mask;                //!< The enabled channels  
  uint8_t buffer_mask;                 //!< The buffer mask at time of read out (do we want this?)   
} nuphase_header_t; 

/**nuphase event body.
 * Holds waveforms. Note that although the buffer length may vary, in memory
 * we always hold max_buffer_size (2048) . Memory is cheap right. Even on the beaglebone, 16KB is no big deal?  (at least, cheaper than dynamic allocation, maybe?) 
 *
 */ 
typedef struct nuphase_event
{
  uint64_t event_number;  //!< The event number. Should match event header.  
  uint16_t buffer_length; //!< The buffer length that is actually filled. Also available in event header. 
  uint8_t  data[NP_NUM_CHAN][NP_MAX_WAVEFORM_LENGTH]; //!< The waveform data. Only the first buffer_length bytes of each are important. 
} nuphase_event_t; 


/** write this header to file. The size will be different than sizeof(nuphase_header_t). Returns 0 on success. */
int nuphase_header_write(FILE * f, const nuphase_header_t * h); 

/** write this header to compressed file. The size will be different than sizeof(nuphase_header_t). Returns 0 on success. */
int nuphase_header_gzwrite(gzFile f, const nuphase_header_t * h); 

/** read this header from file. The size will be different than sizeof(nuphase_header_t). Returns 0 on success. */ 
int nuphase_header_read(FILE * f, nuphase_header_t * h); 

/** read this header from compressed file. The size will be different than sizeof(nuphase_header_t). Returns 0 on success. */ 
int nuphase_header_gzread(gzFile  f, nuphase_header_t * h); 

/** Write the event body to a file. Returns 0 on success. The number of bytes written is not sizeof(nuphase_event_t). */ 
int nuphase_event_write(FILE * f, const nuphase_event_t * ev); 

/** Read the event body from a file. Returns 0 on success. The number of bytes read is not sizeof(nuphase_event_t). */ 
int nuphase_event_read(FILE * f, nuphase_event_t * ev); 

/** Write the event body to a compressed file. Returns 0 on success. The number of bytes written is not sizeof(nuphase_event_t). */ 
int nuphase_event_gzwrite(gzFile f, const nuphase_event_t * ev); 

/** Read the event body from a compressed file. Returns 0 on success. The number of bytes read is not sizeof(nuphase_event_t). */ 
int nuphase_event_gzread(gzFile f, nuphase_event_t * ev); 

#endif
