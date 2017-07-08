#ifndef _nuphase_h
#define _nuphase_h


#include <time.h>
#include <stdint.h>


#define NP_NUM_CHAN 8 

/* Configuration options, sent to fpga */ 

typedef struct nuphase_config
{
//TODO  uint16_t buffer_length; 

  int dummy;
} nuphase_config_t; 



/* stuff like scalers, etc. */ 
typedef struct nuphase_status
{
  //TODO

  int dummy;

}nuphase_status_t; 


typedef struct nuphase_version
{
  uint32_t ver; 
  uint32_t date;
  uint64_t dna;
} nuphase_version_t; 
 


typedef struct nuphase_header 
{
  //TODO
  int dummy;
} nuphase_header_t; 



typedef struct nuphase_event
{
  //TODO
  nuphase_header_t header;
  uint8_t * data[NP_NUM_CHAN]; //8 channels of length buffer length
} nuphase_event_t; 





#endif
