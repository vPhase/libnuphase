#ifndef _nuphasehk_h 
#define _nuphasehk_h 

#include "nuphase.h" 

/* nuphase asps readout/power */ 






/** There are three different ways can talk to the ASPS-DAQ, although they are not all useful for everything. */ 
typedef enum nuphase_asps_method
{
  NP_ASPS_HTTP, 
  NP_ASPS_UART 
} nuphase_asps_method_t; 

int nuphase_update_hk(nuphase_hk_t * hk, nuphase_asps_method_t method); 

int nuphase_set_power_state ( nuphase_power_state_t state, nuphase_asps_method_t method); 





#endif
