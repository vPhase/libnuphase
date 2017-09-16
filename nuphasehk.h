#ifndef _nuphasehk_h 
#define _nuphasehk_h 

#include "nuphase.h" 

/** \file nuphasehk.h
 * 
 *  NuPhase housekeeping functions. 
 *
 *   To get current housekeeping info, use the nuphase_hk method. 
 *
 *   To set the ASPS power state, use nuphase_set_asps_power_state
 *
 *   To set the GPIO 
 *
 *  Cosmin Deaconu
 *  <cozzyd@kicp.uchicago.edu> 
 *
 */

/** There are two  different ways can talk to the ASPS-DAQ. SERIAL should be faster, but HTTP is a potential
 * backup in case something goes wrong with the serial connection */ 
typedef enum nuphase_asps_method
{
  NP_ASPS_HTTP, 
  NP_ASPS_SERIAL 
} nuphase_asps_method_t; 

/** These settings all have sensible defaults, 
 * but you may choose to initialize with other values if necessary */; 
typedef struct nuphase_hk_settings
{
  int master_temperature_ain; // The analog pin used for the master board temperature sensor
  int slave_temperature_ain; // The analog pin used for the slave board temperature sensor
  int master_fpga_power_gpio;   //the gpio pin controlling the power for the master fpga (i.e everything but the temp sensor) 
  int slave_fpga_power_gpio;    //the gpio pin controlling the power for the slave fpga (i.e. everything but the temp sensors) 
  nuphase_fpga_power_state_t init_state;   // The initial state of the FPGA pins. Default is on (but they'll still be off if they're not enabled on the ASPS-DAQ) 
  const char * asps_serial_device;         // The serial device to use to communicate with the ASPS-DAQ via serial ( default: /dev/ttyUSB0 )
  const char * asps_address;               // The hostname or IP address to communicate with the ASPS-DAQ via http (Default asps-daq, which may be set in /etc/hosts to something useful) 
} nuphase_hk_settings_t; 

/** intialize the hk with the chosen settings. If you just want to use the defaults, you 
 * can go ahead and use nuphase_hk/ nuphase_set_asps_power_state / nuphase_set_fpga_power_state 
 * and it will be initialized when necessary with the defaults */ 
int nuphase_hk_init(const nuphase_hk_settings_t * settings); 

/* initialize nuphase_hk_settings_t  with defaults. Do this if you only want to change
 * some things but not all things.  */
void nuphase_hk_settings_init(nuphase_hk_settings_t * settings); 

/** Fills in this hk struct, using the specified method to communicate with the ASPS-DAQ */ 
int nuphase_hk(nuphase_hk_t * hk, nuphase_asps_method_t method); 

/** Set the ASPS power state, using the chosen method. These are all the power switches controlled by the ASPS-DAQ */ 
int nuphase_set_asps_power_state ( nuphase_asps_power_state_t state, nuphase_asps_method_t method); 

/** Set the FPGA power state. These are controlled by a GPIO. For the FPGA's to be on, the relevant ASPS power state must also be enabled */ 
int nuphase_set_fpga_power_state ( nuphase_fpga_power_state_t state); 

#endif
