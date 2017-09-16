#ifndef _nuphase_h
#define _nuphase_h

#ifdef __cplusplus
extern "C" { 
#endif

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
 * In particular, you should use the utility functions here to read/write binary
 * nuphase data from disk as it handles the versioning properly. 
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

//master + slave 
#define NP_MAX_BOARDS 2  

/** The number of trigger beams available */ 
#define NP_NUM_BEAMS 15 

#define NP_NUM_SCALERS 3

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
  NP_TRIG_NONE,   //<! Triggered by nothing (should never happen but if it does it's a bad sign1) 
  NP_TRIG_SW,    //!< triggered by software (force trigger)  
  NP_TRIG_RF,    //!< triggered by input wavecforms
  NP_TRIG_EXT    //!< triggered by external trigger 
} nuphase_trig_type_t; 

/** in memory layout of nuphase event headers. 
 *
 * STILL PRELIMINARY 
 *
 * On-disk layout is different and opaque, you must use nuphase_header_read() / nuphase_header_write() to write
 * to disk 
 *
 * I refrained from reducing the number of bits for various things because they will presumably get compressed away anyway. 
 *
 * If not, we can do that in the conversion to the on-disk format. 
 *
 */
typedef struct nuphase_header 
{
  uint64_t event_number;                         //!< A unique identifier for this event. If only one board, will match readout number. Otherwise, might skip if the boards are out of sync. 
  uint64_t trig_number;                          //!< the sequential (since reset) trigger number assigned to this event. 
  uint16_t buffer_length;                        //!< the buffer length. Stored both here and in the event. 
  uint16_t pretrigger_samples;                   //!< Number of samples that are pretrigger
  uint32_t readout_time[NP_MAX_BOARDS];          //!< CPU time of readout, seconds
  uint32_t readout_time_ns[NP_MAX_BOARDS];       //!< CPU time of readout, nanoseconds 
  uint64_t trig_time[NP_MAX_BOARDS];             //!< Board trigger time (raw units) 
  uint32_t approx_trigger_time;                  //!< Board trigger time converted to real units (approx secs), master only
  uint32_t approx_trigger_time_nsecs;            //!< Board trigger time converted to real units (approx nnsecs), master only
  uint16_t triggered_beams;                      //!< The beams that triggered 
  uint16_t beam_mask;                            //!< The enabled beams
  uint32_t beam_power[NP_NUM_BEAMS];             //!< The power in each beam at the trigger time
  uint32_t deadtime[NP_MAX_BOARDS];              //!< ??? Will we have this available? If so, this will be a fraction. (store for slave board as well) 
  uint8_t buffer_number;                         //!< the buffer number (do we need this?) 
  uint8_t channel_mask;                          //!< The channels allowed to participate in the trigger
  uint8_t channel_read_mask[NP_MAX_BOARDS];      //!< The channels actually read
  uint8_t channel_overflow;                      //!< Bitmask of channels that overflowed the 5 bits 
  uint8_t buffer_mask;                           //!< The buffer mask at time of read out (do we want this?)   
  uint8_t board_id[NP_MAX_BOARDS];               //!< The board number assigned at startup. If board_id[1] == 0, no slave. 
  nuphase_trig_type_t trig_type;                 //!< The trigger type? 
  uint8_t calpulser;                             //!< Was the calpulser on? 
  uint8_t sync_problem;                          //!< Various sync problems. TODO convert to enum 
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
  uint8_t board_id[NP_MAX_BOARDS];     //!< The board number assigned at startup. If the second board_id is zero, that indicates there is no slave device. 
  uint8_t  data[NP_MAX_BOARDS][NP_NUM_CHAN][NP_MAX_WAVEFORM_LENGTH]; //!< The waveform data. Only the first buffer_length bytes of each are important. The second array is only filled if there is a slave-device. 
} nuphase_event_t; 


/** nuphase status. 
 * Holds scalers, deadtime, and maybe some other things 
 **/




typedef enum nuphase_scaler_type
{
  SCALER_SLOW, 
  SCALER_SLOW_GATED,
  SCALER_FAST
} nuphase_scaler_type_t; 


typedef struct nuphase_status
{
  uint16_t global_scalers[NP_NUM_SCALERS];
  uint16_t beam_scalers[NP_NUM_SCALERS][NP_NUM_BEAMS];  //!< The scaler for each beam (12 bits) 
  uint32_t deadtime;               //!< The deadtime fraction (units tbd) 
  uint32_t readout_time;           //!< CPU time of readout, seconds
  uint32_t readout_time_ns;        //!< CPU time of readout, nanoseconds 
  uint8_t board_id;               //!< The board number assigned at startup. 

} nuphase_status_t; 


/** bitmask of what is on , according to ASPS-DAQ*/ 
typedef enum nuphase_asps_power_state
{
  NP_POWER_FRONTEND = 1,
  NP_POWER_SBC      = 2, 
  NP_POWER_SLAVE    = 4, 
  NP_POWER_MASTER   = 8, 
  NP_POWER_SWITCH   = 16  

} nuphase_asps_power_state_t; 

/* Power state of FPGA (the board can be on but the FPGA off) */ 
typedef enum nuphase_fpga_power_state
{
  NP_FPGA_POWER_MASTER = 1, 
  NP_FPGA_POWER_SLAVE = 2 
} nuphase_fpga_power_state_t; 


typedef struct nuphase_hk
{
  uint32_t unixTime; 
  uint16_t unixTimeMillisecs; 
  int8_t temp_master;  //C, or -128 if off
  int8_t temp_slave;   //C, or -128 if off 
  int8_t temp_case; 
  int8_t temp_asps_uc; 
  uint16_t current_master; //mA
  uint16_t current_slave; 
  uint16_t current_frontend; 
  uint16_t current_sbc; 
  uint16_t current_switch; 
  nuphase_asps_power_state_t on_state : 8; 
  nuphase_asps_power_state_t fault_state : 8; 
  nuphase_fpga_power_state_t fpga_state : 8; 
  uint32_t disk_space_kB; 
  uint32_t free_mem_kB;  
} nuphase_hk_t; 


/** print the status  prettily */
int nuphase_status_print(FILE *f, const nuphase_status_t * st) ; 

/** print the header  prettily */
int nuphase_header_print(FILE *f, const nuphase_header_t * h) ; 

/** print the event prettily. The separator character will be used to separate different fields so you can dump it into a spreadsheet or something */
int nuphase_event_print(FILE *f, const nuphase_event_t * ev, char sep) ; 

/** Print the HK status pretilly */ 
int nuphase_hk_print(FILE * f, const nuphase_hk_t * hk); 

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

/** Write the status to a file. Returns 0 on success. The number of bytes written is not sizeof(nuphase_status_t). */ 
int nuphase_status_write(FILE * f, const nuphase_status_t * ev); 

/** Read the statusbody from a file. Returns 0 on success. The number of bytes read is not sizeof(nuphase_status_t). */ 
int nuphase_status_read(FILE * f, nuphase_status_t * ev); 

/** Write the status to a compressed file. Returns 0 on success. The number of bytes written is not sizeof(nuphase_status_t). */ 
int nuphase_status_gzwrite(gzFile f, const nuphase_status_t * ev); 

/** Read the status from a compressed file. Returns 0 on success. The number of bytes read is not sizeof(nuphase_status_t). */ 
int nuphase_status_gzread(gzFile f, nuphase_status_t * ev); 

/** write this hk to file. The size will be different than sizeof(nuphase_hk_t). Returns 0 on success. */
int nuphase_hk_write(FILE * f, const nuphase_hk_t * h); 

/** write this hk to compressed file. The size will be different than sizeof(nuphase_hk_t). Returns 0 on success. */
int nuphase_hk_gzwrite(gzFile f, const nuphase_hk_t * h); 

/** read this hk from file. The size will be different than sizeof(nuphase_hk_t). Returns 0 on success. */ 
int nuphase_hk_read(FILE * f, nuphase_hk_t * h); 

/** read this hk from compressed file. The size will be different than sizeof(nuphase_hk_t). Returns 0 on success. */ 
int nuphase_hk_gzread(gzFile  f, nuphase_hk_t * h); 



#ifdef __cplusplus
}
#endif
#endif
