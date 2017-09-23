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
 *   To set the GPIO power states  
 *
 *
 *  This library uses static internal buffers so it is not thread-safe nor reentrant! 
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
 * but you may choose to initialize with other values if necessary */ 
typedef struct nuphase_hk_settings
{
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

/** Set the GPIO power state. For the FPGA's to be on, the relevant ASPS power state must also be enabled
 * Note that even for things with inverted state (active low instead of active high), you should use the 
 * logical state here. 
 *
 * The mask allows you to only set some of the pins (and leave the others unchanged) 
 **/ 
int nuphase_set_gpio_power_state ( nuphase_gpio_power_state_t state, nuphase_gpio_power_state_t mask); 

/* Gets the pid goal of the heater via the method */ 
int nuphase_get_asps_heater_current(nuphase_asps_method_t method); 

/* Sets the pid goal of the heater using the method. Note that the heater on after end time
 * might set this from underneath us */ 
int nuphase_set_asps_heater_current(int current_mA, nuphase_asps_method_t method); 



#endif
