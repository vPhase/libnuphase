#include "nuphase.h" 
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <time.h> 

//these need to be incremented if the structs change incompatibly
//and then generic_*_read must be updated to delegate appropriately. 
#define NUPHASE_HEADER_VERSION 2 
#define NUPHASE_EVENT_VERSION 2 
#define NUPHASE_STATUS_VERSION 2 
#define NUPHASE_HK_VERSION 0 

#define NUPHASE_HEADER_MAGIC 0xa1 
#define NUPHASE_EVENT_MAGIC  0xd0 
#define NUPHASE_STATUS_MAGIC 0x57 
#define NUPHASE_HK_MAGIC     0x79 


//TODO there are apparently much faster versions of these 

static uint16_t stupid_fletcher16_append(int N, const void * vbuf, uint16_t append) 

{
  int i; 
  uint16_t sum1 = append  & 0xff; 
  uint16_t sum2 = append >> 8;; 
  uint8_t * buf = (uint8_t*) vbuf; 

  for (i = 0; i < N; i++)
  {
    sum1 =  (sum1 +buf[i]) % 255; 
    sum2 += (sum1 + sum2) % 255;;
  }

  return sum1 | (sum2 << 8) ; 
}

static uint16_t stupid_fletcher16(int N, const void * buf) 
{
  return stupid_fletcher16_append(N, buf, 0); 
}




/* we'll handle (gz|f)(read|write) the same 
 * with a little silly trickery */ 
struct generic_file
{
  enum { STDIO, ZLIB } type; 
  union 
  {
    FILE * f;
    gzFile gzf;
  } handle; 
}; 

static int generic_read(struct generic_file gf, int n, void * buf) 
{
  switch(gf.type)
  {
    case STDIO: 
      return fread(buf,1,n, gf.handle.f); 
    case ZLIB: 
      return gzread(gf.handle.gzf,buf,n); 
    default:
      return -1; 
  }
}



static int generic_write(struct generic_file gf, int n, const void * buf) 
{
  switch(gf.type)
  {
    case STDIO: 
      return fwrite(buf, 1,n,gf.handle.f); 
    case ZLIB: 
      return gzwrite(gf.handle.gzf,buf,n); 
    default:
      return -1; 
  }
}



struct packet_start
{
  uint8_t magic; 
  uint8_t ver; 
  uint16_t cksum; 
};

// takes care of the odious task of reading in the packet start 
static int packet_start_read( struct generic_file gf, struct packet_start * start, uint8_t expected_magic, uint8_t maximum_version)
{
  int got; 
  got = generic_read(gf, sizeof(start->magic), &start->magic); 
  if (got != sizeof(start->magic)) return NP_ERR_NOT_ENOUGH_BYTES; 

  if (start->magic != expected_magic)
  {
    fprintf(stderr,"Bad magic byte. Expected 0x%x, got 0x%x\n", expected_magic, start->magic); 
    return NP_ERR_WRONG_TYPE; 
  }


  got = generic_read(gf, sizeof(start->ver), &start->ver); 
  if (got != sizeof(start->ver)) 
  {
    fprintf(stderr,"Did not get enough start bytes\n"); 
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (start->ver > maximum_version) 
  {
    fprintf(stderr,"Version %d exceeds maximum %d\n", start->ver , maximum_version); 
    return NP_ERR_BAD_VERSION; 
  }

  got = generic_read(gf,sizeof(start->cksum), &start->cksum); 

  if (got != sizeof(start->cksum))
  {
    fprintf(stderr,"Did not get enough checksum bytes\n"); 
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  return 0; 
}



typedef struct nuphase_header_v0
{
  uint64_t event_number;         
  uint64_t trig_number;                          
  uint16_t buffer_length;                        
  uint16_t pretrigger_samples;                   
  uint32_t readout_time;                         
  uint32_t readout_time_ns;                      
  uint64_t trig_time;                            
  uint32_t approx_trigger_time;                  
  uint32_t approx_trigger_time_nsecs;            
  uint16_t triggered_beams;                      
  uint16_t beam_mask;                            
  uint32_t beam_power[NP_NUM_BEAMS];             
  uint32_t deadtime;                             
  uint8_t buffer_number;                         
  uint8_t channel_mask;                          
  uint8_t channel_overflow;                      
  uint8_t buffer_mask;                           
  uint8_t board_id;                              
  nuphase_trig_type_t trig_type;                 
  uint8_t calpulser;                             
} nuphase_header_v0_t; 

typedef struct nuphase_header_v1
{
  uint64_t event_number;         
  uint64_t readout_number;         
  uint64_t trig_number;                          
  uint16_t buffer_length;                        
  uint16_t pretrigger_samples;                   
  uint32_t readout_time;                         
  uint32_t readout_time_ns;                      
  uint64_t trig_time;                            
  uint32_t approx_trigger_time;                  
  uint32_t approx_trigger_time_nsecs;            
  uint16_t triggered_beams;                      
  uint16_t beam_mask;                            
  uint32_t beam_power[NP_NUM_BEAMS];             
  uint32_t deadtime;                             
  uint8_t buffer_number;                         
  uint8_t channel_mask;                          
  uint8_t channel_overflow;                      
  uint8_t buffer_mask;                           
  uint8_t board_id;                              
  nuphase_trig_type_t trig_type;                 
  uint8_t calpulser;                             
} nuphase_header_v1_t; 


/* Offsets from start of structs for headers */ 
const int nuphase_header_sizes []=  { sizeof(nuphase_header_v0_t), sizeof(nuphase_header_v1_t), sizeof(nuphase_header_t) }; 


typedef struct nuphase_event_v0
{
  uint64_t event_number; 
  uint16_t buffer_length; 
  uint8_t  data[NP_NUM_CHAN][NP_MAX_WAVEFORM_LENGTH]; 
  uint8_t board_id;     
} nuphase_event_v0_t; 

typedef struct nuphase_event_v1
{
  uint64_t event_number; 
  uint64_t readout_number; 
  uint16_t buffer_length; 
  uint8_t  data[NP_NUM_CHAN][NP_MAX_WAVEFORM_LENGTH]; 
  uint8_t board_id;     
} nuphase_event_v1_t; 

typedef struct nuphase_status_v0
{
  uint16_t global_scalers[NP_NUM_SCALERS];
  uint16_t beam_scalers[NP_NUM_SCALERS][NP_NUM_BEAMS];  //!< The scaler for each beam (12 bits) 
  uint32_t deadtime;               //!< The deadtime fraction (units tbd) 
  uint32_t readout_time;           //!< CPU time of readout, seconds
  uint32_t readout_time_ns;        //!< CPU time of readout, nanoseconds 
  uint32_t trigger_thresholds[NP_NUM_BEAMS]; //!< The trigger thresholds  
  uint64_t latched_pps_time;      //!< A timestamp corresponding to a pps time 
  uint8_t board_id;               //!< The board number assigned at startup. 

} nuphase_status_v0_t; 




static void convert_header(int version, nuphase_header_t * new_header, const void * old_header) 
{
  if (version == NUPHASE_HEADER_VERSION) 
  {
      memcpy(new_header,old_header,sizeof(*new_header)); 
  }
  else
  {
      fprintf(stderr,"unknown version: %d\n", version); 
      memset(new_header,0,sizeof(*new_header)); 
  }

}


/* The on-disk format is just packet_start followed by the newest version of the
 * the header struct.  Every time the version changes,if we have data we care about, 
 * we need to increment the version. 
 */

static int nuphase_header_generic_write(struct generic_file gf, const nuphase_header_t *h)
{
  struct packet_start start; 
  int written; 
  start.magic = NUPHASE_HEADER_MAGIC; 
  start.ver = NUPHASE_HEADER_VERSION; 
  start.cksum = stupid_fletcher16(sizeof(nuphase_header_t), h); 

  written = generic_write(gf, sizeof(start), &start); 
  if (written != sizeof(start)) 
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  written = generic_write(gf, sizeof(nuphase_header_t), h); 
  
  if (written != sizeof(nuphase_header_t))
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  return 0; 
}



static int nuphase_header_generic_read(struct generic_file gf, nuphase_header_t *h) 
{
  struct packet_start start; 
  int got; 
  int wanted; 
  uint16_t cksum; 

  got = packet_start_read(gf, &start, NUPHASE_HEADER_MAGIC, NUPHASE_HEADER_VERSION); 
  if (got) return got; 

  void * oldmem = 0;

  switch(start.ver) 
  {
    //add cases here if necessary 
    case 0: 
      wanted = nuphase_header_sizes[start.ver]; 
      oldmem = malloc(wanted); 
      got = generic_read(gf, wanted, oldmem); 
      cksum = stupid_fletcher16(wanted, oldmem); 
      convert_header(start.ver,h, oldmem); 
      free(oldmem); 
      break; 
    case NUPHASE_HEADER_VERSION: //this is the most recent header!
      wanted = sizeof(nuphase_header_t); 
      got = generic_read(gf, wanted, h); 
      cksum = stupid_fletcher16(wanted, h); 
      break; 
    default: 
    return NP_ERR_BAD_VERSION; 
  }

  if (wanted!=got)
  {
    fprintf(stderr,"not enough bytes\n"); 
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (cksum != start.cksum) 
  {
    fprintf(stderr,"cksum problem\n"); 
    return NP_ERR_CHECKSUM_FAILED; 
  }

  return 0; 
}


/* The on-disk format is just packet_start followed by the newest version of the
 * the event struct. Note that we only write (and compute the checksum for) buffer length bytes for each event. 
 *
 * very time the version changes,if we have data we care about, 
 * we need to increment the version. 
 */

static int nuphase_event_generic_write(struct generic_file gf, const nuphase_event_t *ev)
{
  struct packet_start start; 
  int written; 
  int i,ibd; 
  start.magic = NUPHASE_EVENT_MAGIC; 
  start.ver = NUPHASE_EVENT_VERSION; 

  start.cksum = stupid_fletcher16(sizeof(ev->event_number), &ev->event_number); 
  start.cksum = stupid_fletcher16_append(sizeof(ev->buffer_length), &ev->buffer_length,start.cksum); 
  start.cksum = stupid_fletcher16_append(sizeof(ev->board_id), &ev->board_id, start.cksum); 

  for (ibd = 0; ibd <NP_MAX_BOARDS ; ibd++)
  {
    if (!ev->board_id[ibd]) continue; 
    for (i = 0; i < NP_NUM_CHAN; i++) 
    {
     start.cksum = stupid_fletcher16_append(ev->buffer_length, ev->data[ibd][i], start.cksum); 
    }
  }


  written = generic_write(gf, sizeof(start), &start); 
  if (written != sizeof(start)) 
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  written = generic_write(gf, sizeof(ev->event_number), &ev->event_number); 

  if (written != sizeof(ev->event_number))
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }


  written = generic_write(gf, sizeof(ev->buffer_length), &ev->buffer_length); 
  if (written != sizeof(ev->buffer_length))
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  written = generic_write(gf, sizeof(ev->board_id), &ev->board_id); 

  if (written != sizeof(ev->board_id)) 
  {
      return NP_ERR_NOT_ENOUGH_BYTES; 
  }
 
  for (ibd = 0; ibd < NP_MAX_BOARDS; ibd++)
  {
    if (!ev->board_id[ibd]) continue; 
    for (i = 0; i <NP_NUM_CHAN; i++)
    {
      written = generic_write(gf, ev->buffer_length, &ev->data[ibd][i][0]); 
      if (written != ev->buffer_length) 
      {
        return NP_ERR_NOT_ENOUGH_BYTES; 
      }
    }
  }

  return 0; 
}


static int nuphase_event_generic_read(struct generic_file gf, nuphase_event_t *ev) 
{
  struct packet_start start; 
  int got; 
  int wanted; 
  uint16_t cksum; 
  int i; 

  got = packet_start_read(gf, &start, NUPHASE_EVENT_MAGIC, NUPHASE_EVENT_VERSION); 
  if (got) return got; 


  //add additional cases if necessary for compatibility
  //
  if (start.ver == NUPHASE_EVENT_VERSION) 
  {
      wanted = sizeof(ev->event_number); 
      got = generic_read(gf, wanted, &ev->event_number); 
      if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
      cksum = stupid_fletcher16(wanted, &ev->event_number); 

      wanted = sizeof(ev->buffer_length); 
      got = generic_read(gf, wanted, &ev->buffer_length); 
      if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
      cksum = stupid_fletcher16_append(wanted, &ev->buffer_length,cksum); 

      wanted = sizeof(ev->board_id); 
      got = generic_read(gf, wanted, &ev->board_id); 
      if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
      cksum = stupid_fletcher16_append(wanted, &ev->board_id,cksum); 

      int ibd; 
      for (ibd = 0; ibd <NP_MAX_BOARDS; ibd++)
      {
        if (!ev->board_id[ibd]) 
        {
          memset(ev->data[ibd],0, sizeof(ev->data[ibd])); 
          continue; 
        }

        for (i = 0; i < NP_NUM_CHAN; i++)
        {
          wanted = ev->buffer_length; 
          got = generic_read(gf, wanted, ev->data[ibd][i]); 
          if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
          cksum = stupid_fletcher16_append(wanted, ev->data[ibd][i], cksum); 

          // zero out the rest of the memory 
          memset(ev->data[ibd][i] + wanted, 0, NP_MAX_WAVEFORM_LENGTH - wanted); 
        }
      }

  }
  
  else
  {
    fprintf(stderr,"Unimplemented version: %d\n", start.ver); 
    return NP_ERR_BAD_VERSION; 
  }

  if (cksum != start.cksum) 
  {
    return NP_ERR_CHECKSUM_FAILED; 
  }

  return 0; 
}

/** The on-disk format is packet_start followed by the newest version of the status struct. 
 *
 * Note that the implementation of status and header are basically the same right now... but that might
 * change if one of the versions changes. 
 */
static int nuphase_status_generic_write(struct generic_file gf, const nuphase_status_t *st) 
{
  struct packet_start start; 
  int written; 
  start.magic = NUPHASE_STATUS_MAGIC; 
  start.ver = NUPHASE_STATUS_VERSION; 
  start.cksum = stupid_fletcher16(sizeof(nuphase_status_t), st); 

  written = generic_write(gf, sizeof(start), &start); 
  if (written != sizeof(start)) 
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  written = generic_write(gf, sizeof(nuphase_status_t), st); 
  
  if (written != sizeof(nuphase_status_t))
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  return 0; 
}

static int nuphase_status_generic_read(struct generic_file gf, nuphase_status_t *st) 
{
  struct packet_start start; 
  int got; 
  int wanted; 
  uint16_t cksum; 
  nuphase_status_v0_t v0; 


  got = packet_start_read(gf, &start, NUPHASE_STATUS_MAGIC, NUPHASE_STATUS_VERSION); 
  if (got) return got; 

  switch(start.ver) 
  {
    //add cases here if necessary 
    case 0: 
      wanted = sizeof(nuphase_status_v0_t); 
      got = generic_read(gf, wanted, &v0); 
      cksum = stupid_fletcher16(wanted, &v0); 
      memcpy(st->global_scalers, v0.global_scalers, sizeof(v0.global_scalers)); 
      memcpy(st->beam_scalers, v0.beam_scalers, sizeof(v0.beam_scalers)); 
      st->deadtime = v0.deadtime; 
      st->readout_time = v0.readout_time; 
      st->readout_time_ns = v0.readout_time; 
      memcpy(st->trigger_thresholds, v0.trigger_thresholds, sizeof(v0.trigger_thresholds)); 
      st->board_id = v0.board_id; 
      st->latched_pps_time = 0; 
      break; 
    case NUPHASE_STATUS_VERSION: //this is the most recent status!
      wanted = sizeof(nuphase_status_t); 
      got = generic_read(gf, wanted, st); 
      cksum = stupid_fletcher16(wanted, st); 
      break; 
    default: 
    return NP_ERR_BAD_VERSION; 
  }

  if (wanted!=got)
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (cksum != start.cksum) 
  {
    return NP_ERR_CHECKSUM_FAILED; 
  }

  return 0; 
}

static int nuphase_hk_generic_write(struct generic_file gf, const nuphase_hk_t *hk)
{
  struct packet_start start; 
  int written; 
  start.magic = NUPHASE_HK_MAGIC; 
  start.ver = NUPHASE_HK_VERSION; 
  start.cksum = stupid_fletcher16(sizeof(nuphase_hk_t), hk); 

  written = generic_write(gf, sizeof(start), &start); 
  if (written != sizeof(start)) 
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  written = generic_write(gf, sizeof(nuphase_hk_t), hk); 
  
  if (written != sizeof(nuphase_hk_t))
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  return 0; 
}

static int nuphase_hk_generic_read(struct generic_file gf, nuphase_hk_t *hk) 
{
  struct packet_start start; 
  int got; 
  int wanted; 
  uint16_t cksum; 

  got = packet_start_read(gf, &start, NUPHASE_HK_MAGIC, NUPHASE_HK_VERSION); 
  if (got) return got; 

  switch(start.ver) 
  {
    //add cases here if necessary 
    case NUPHASE_HK_VERSION: //this is the most recent hk!
      wanted = sizeof(nuphase_hk_t); 
      got = generic_read(gf, wanted, hk); 
      cksum = stupid_fletcher16(wanted, hk); 
      break; 
    default: 
    return NP_ERR_BAD_VERSION; 
  }

  if (wanted!=got)
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (cksum != start.cksum) 
  {
    return NP_ERR_CHECKSUM_FAILED; 
  }

  return 0; 
}



/* 
 * these should all probably be generated by a macro instead of my copy-paste job...
 **/

int nuphase_event_write(FILE * f, const nuphase_event_t * ev) 
{
  struct generic_file gf=  { .type = STDIO, .handle.f = f }; 
  return nuphase_event_generic_write(gf, ev); 
}

int nuphase_event_gzwrite(gzFile f, const nuphase_event_t * ev) 
{
  struct generic_file gf=  { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_event_generic_write(gf, ev); 
}

int nuphase_event_read(FILE * f, nuphase_event_t * ev) 
{
  struct generic_file gf=  { .type = STDIO, .handle.f = f }; 
  return nuphase_event_generic_read(gf, ev); 
}

int nuphase_event_gzread(gzFile f, nuphase_event_t * ev) 
{
  struct generic_file gf=  { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_event_generic_read(gf, ev); 
}

int nuphase_status_write(FILE * f, const nuphase_status_t * ev) 
{
  struct generic_file gf=  { .type = STDIO, .handle.f = f }; 
  return nuphase_status_generic_write(gf, ev); 
}

int nuphase_status_gzwrite(gzFile f, const nuphase_status_t * ev) 
{
  struct generic_file gf=  { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_status_generic_write(gf, ev); 
}

int nuphase_status_read(FILE * f, nuphase_status_t * ev) 
{
  struct generic_file gf=  { .type = STDIO, .handle.f = f }; 
  return nuphase_status_generic_read(gf, ev); 
}

int nuphase_status_gzread(gzFile f, nuphase_status_t * ev) 
{
  struct generic_file gf=  { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_status_generic_read(gf, ev); 
}

int nuphase_header_write(FILE * f, const nuphase_header_t * h) 
{
  struct generic_file gf = { .type = STDIO, .handle.f = f }; 
  return nuphase_header_generic_write(gf, h); 
}

int nuphase_header_gzwrite(gzFile f, const nuphase_header_t * h) 
{
  struct generic_file gf = { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_header_generic_write(gf, h); 
}

int nuphase_header_read(FILE * f, nuphase_header_t * h) 
{
  struct generic_file gf = { .type = STDIO, .handle.f = f }; 
  return nuphase_header_generic_read(gf, h); 
}

int nuphase_header_gzread(gzFile f, nuphase_header_t * h) 
{
  struct generic_file gf = { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_header_generic_read(gf, h); 
}

int nuphase_hk_write(FILE * f, const nuphase_hk_t * h) 
{
  struct generic_file gf = { .type = STDIO, .handle.f = f }; 
  return nuphase_hk_generic_write(gf, h); 
}

int nuphase_hk_gzwrite(gzFile f, const nuphase_hk_t * h) 
{
  struct generic_file gf = { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_hk_generic_write(gf, h); 
}

int nuphase_hk_read(FILE * f, nuphase_hk_t * h) 
{
  struct generic_file gf = { .type = STDIO, .handle.f = f }; 
  return nuphase_hk_generic_read(gf, h); 
}

int nuphase_hk_gzread(gzFile f, nuphase_hk_t * h) 
{
  struct generic_file gf = { .type = ZLIB, .handle.gzf = f }; 
  return nuphase_hk_generic_read(gf, h); 
}




/* pretty prints */ 

int nuphase_status_print(FILE *f, const nuphase_status_t *st)
{
  int i ; 
  struct tm * tim; 
  char timstr[128]; 
  time_t t = st->readout_time;
  tim = gmtime((time_t*) &t); 
  strftime(timstr,sizeof(timstr), "%Y-%m-%d %H:%M:%S", tim);  
  fprintf(f,"NuPhase Board 0x%x Status (read at %s.%09d UTC)\n", st->board_id, timstr, st->readout_time_ns); 

  fprintf(f,"\t which \t 0.1 Hz, gated 0.1Hz, 1 Hz, threshold\n"); 
  fprintf(f,"\tGLOBAL: \t%u \t%u \t%u\n", st->global_scalers[SCALER_SLOW], st->global_scalers[SCALER_SLOW_GATED], st->global_scalers[SCALER_FAST]); 
  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    fprintf(f,"\tBEAM %d: \t%u \t%u \t%u \t%u \n",i, st->beam_scalers[SCALER_SLOW][i], st->beam_scalers[SCALER_SLOW_GATED][i], st->beam_scalers[SCALER_FAST][i], st->trigger_thresholds[i] ); 
  }
  return 0; 
}

static const char * trig_type_names[4]  = { "NONE", "SW", "RF" ,"EXT" } ; 


int nuphase_header_print(FILE *f, const nuphase_header_t *hd)
{
  int i; 
  struct tm* tim; 
  time_t t; 
  char timstr[128]; 

  fprintf(f, "EVENT_NUMBER %"PRIu64"\n", hd->event_number ); 
  fprintf(f, "\t%s TRIGGER\n", trig_type_names[hd->trig_type]); 
  fprintf(f,  "\ttrig num: %"PRIu64" boards: [%d,%d], sync_problem: %x\n", hd->trig_number, hd->board_id[0], hd->board_id[1], hd->sync_problem); 
  fprintf(f, "\tbuf len: %u ; pretrig: %u\n", hd->buffer_length, hd->pretrigger_samples); 
  fprintf(f,"\tbuf num: %u, buf_mask: %x\n", hd->buffer_number, hd->buffer_mask); 
  t = hd->readout_time[0];
  tim = gmtime((time_t*) &t); 
  strftime(timstr,sizeof(timstr), "%Y-%m-%d %H:%M:%S", tim);  
  fprintf(f, "\tbd %d rdout time: %s.%09d UTC\n",hd->board_id[0],timstr, hd->readout_time_ns[0]); 
  if (hd->board_id[1])
  {
    t = hd->readout_time[1];
    tim = gmtime((time_t*) &t); 
    strftime(timstr,sizeof(timstr), "%Y-%m-%d %H:%M:%S", tim);  
    fprintf(f, "\tbd %d rdout time: %s.%09d UTC\n",hd->board_id[1], timstr, hd->readout_time_ns[1]); 
   
  }

  fprintf(f, "\tbd %d trig time (raw): %"PRIu64"\n", hd->board_id[0], hd->trig_time[0]); 

  if (hd->board_id[1])
  {

    fprintf(f, "\tbd %d trig time (raw): %"PRIu64"\n", hd->board_id[1], hd->trig_time[1]); 
  }

  t = hd->approx_trigger_time; 
  tim = gmtime((time_t*) &t); 
  strftime(timstr,sizeof(timstr), "%Y-%m-%d %H:%M:%S", tim);  
  fprintf(f, "\ttrig time (est.): %s.%09d UTC\n",timstr, hd->approx_trigger_time_nsecs); 
  fprintf(f, "\ttrig beams: %x\n", hd->triggered_beams); 
  fprintf(f, "\tenabld beams: %x\n", hd->beam_mask); 
  fprintf(f, "\tbeam powers:\n") ; 
  for (i = 0; i < NP_NUM_BEAMS; i++) 
  {
    fprintf(f,"\t\tB%02d: %u\n", i, hd->beam_power[i]); 
  }
  fprintf(f,"\tprev sec deadtime: (%u, %u)\n", hd->deadtime[0], hd->deadtime[1]); 
  fprintf(f,"\ttrig_channel_mask: %x\n", hd->channel_mask); 
  fprintf(f,"\tchannel_read_mask: (%x,%x)\n", hd->channel_read_mask[0],hd->channel_read_mask[1]); 
  fprintf(f,"\tcalpulser: %s\n", hd->calpulser ? "yes" : "no"); 
  

  return 0; 
}


int nuphase_event_print(FILE *f, const nuphase_event_t *ev, char sep)
{
  int ichan, isamp, ibd ; 
  for (ibd = 0; ibd < NP_MAX_BOARDS; ibd++)
  {
    if (!ev->board_id[ibd]) continue;
    fprintf(f, "EVENT NUMBER:%c %"PRIu64" %c BOARD: %c %d %c LENGTH: %c %d \n", sep,ev->event_number,sep,sep,ev->board_id[ibd], sep,sep,ev->buffer_length ); 
    for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
    {
      for (isamp = 0; isamp < ev->buffer_length; isamp++) 
      {
        fprintf(f, "%d%c", ev->data[ibd][ichan][isamp], isamp < ev->buffer_length - 1 ? sep : '\n'); 
      }
    }
  }

  return 0; 
}

int nuphase_hk_print(FILE * f, const nuphase_hk_t *hk) 
{
  struct tm*  tim; 
  char timstr[128]; 
  time_t t = hk->unixTime;
  tim = gmtime(&t); 
  strftime(timstr,sizeof(timstr), "%Y-%m-%d %H:%M:%S", tim);  
  fprintf(f,"NuPhase HK (at %s.%03d UTC)\n", timstr, hk->unixTimeMillisecs); 
  fprintf(f,"  Temperatures: \n"); 
  if (hk->temp_master > -128)
  {
    fprintf(f,"      MASTER:  %d C\n", hk->temp_master); 
  }
  else
  {
    fprintf(f,"      MASTER: sensor off\n"); 
  }

  if (hk->temp_slave > -128)
  {
    fprintf(f,"      SLAVE :  %d C\n", hk->temp_slave); 
  }
  else
  {
    fprintf(f,"      SLAVE: sensor off\n"); 
  }
  fprintf(f,"      CASE  :  %d C\n", hk->temp_case); 
  fprintf(f,"      ASPSuC:  %d C\n", hk->temp_asps_uc); 
  fprintf(f,"  Power: \n"); 
  fprintf(f,"      MASTER     :  %s (%d mA) \n", (hk->on_state & NP_POWER_MASTER)  ? "ON ":"OFF",  hk->current_master); 
  fprintf(f,"      SLAVE      :  %s (%d mA) \n", (hk->on_state & NP_POWER_SLAVE)  ? "ON ":"OFF",  hk->current_slave); 
  fprintf(f,"      FRNTEND    :  %s (%d mA) \n", (hk->on_state & NP_POWER_FRONTEND)  ? "ON ":"OFF",  hk->current_frontend); 
  fprintf(f,"      SBC        :  %s (%d mA) \n", (hk->on_state & NP_POWER_SBC)  ? "ON ":"OFF",  hk->current_sbc); 
  fprintf(f,"      SWITCH     :  %s (%d mA) \n", (hk->on_state & NP_POWER_SWITCH)  ? "ON ":"OFF",  hk->current_switch); 
  fprintf(f,"      MASTER_FPGA:  %s \n", ( (hk->gpio_state & NP_FPGA_POWER_MASTER) && ( hk->on_state & NP_POWER_MASTER) )  ? "ON ":"OFF"); 
  fprintf(f,"      SLAVE_FPGA :  %s \n", ( (hk->gpio_state & NP_FPGA_POWER_SLAVE) && ( hk->on_state & NP_POWER_SLAVE) )  ? "ON ":"OFF"); 
  fprintf(f,"      SPI        :  %s \n", (hk->gpio_state & NP_SPI_ENABLE)   ? "ON ":"OFF"); 
  fprintf(f,"      DOWNHOLE   :  %s \n", (hk->gpio_state & NP_DOWNHOLE_POWER)   ? "ON ":"OFF"); 
  fprintf(f,"      AUX_HEATER :  %s \n", (hk->gpio_state & NP_AUX_HEATER)   ? "ON ":"OFF"); 
  fprintf(f,"      ASPS_HEATER:  %s (%d mA)\n", hk->asps_heater_current ? " ON" : "OFF", hk->asps_heater_current); 
  fprintf(f,"  SBC: \n"); 
  fprintf(f,"     DISK SPACE: %0.3g MB \n", hk->disk_space_kB /1024.);  
  fprintf(f,"     FREE MEM  : %0.3g MB \n", hk->free_mem_kB   /1024.);  

  return 0; 
}


