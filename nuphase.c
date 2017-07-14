#include "nuphase.h" 

#include <string.h>

//these need to be incremented if the structs change incompatibly
//and then generic_*_read must be updated to delegate appropriately. 
#define NUPHASE_HEADER_VERSION 0 
#define NUPHASE_EVENT_VERSION 0 
#define NUPHASE_STATUS_VERSION 0 

#define NUPHASE_HEADER_MAGIC 0xa1 
#define NUPHASE_EVENT_MAGIC  0xd0 
#define NUPHASE_STATUS_MAGIC 0x57 

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
      return fread(buf,8,n, gf.handle.f); 
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
      return fwrite(buf, 8,n,gf.handle.f); 
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
    return NP_ERR_WRONG_TYPE; 

  got = generic_read(gf, sizeof(start->ver), &start->ver); 
  if (got != sizeof(start->ver)) return NP_ERR_NOT_ENOUGH_BYTES; 

  if (start->ver > maximum_version) 
    return NP_ERR_BAD_VERSION; 

  got = generic_read(gf,sizeof(start->cksum), &start->cksum); 

  if (got != sizeof(start->cksum)) return NP_ERR_NOT_ENOUGH_BYTES; 

  return 0; 
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

  switch(start.ver) 
  {
    //add cases here if necessary 
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
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (cksum != start.cksum) 
  {
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
  int i; 
  start.magic = NUPHASE_EVENT_MAGIC; 
  start.ver = NUPHASE_EVENT_VERSION; 

  start.cksum = stupid_fletcher16(sizeof(ev->event_number), &ev->event_number); 
  start.cksum = stupid_fletcher16_append(sizeof(ev->buffer_length), &ev->buffer_length,start.cksum); 

  for (i = 0; i < NP_NUM_CHAN; i++) 
  {
   start.cksum = stupid_fletcher16_append(ev->buffer_length, ev->data[i], start.cksum); 
  }

  start.cksum = stupid_fletcher16_append(sizeof(ev->board_id), &ev->board_id, start.cksum); 

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

  for (i = 0; i <NP_NUM_CHAN; i++)
  {
    written = generic_write(gf, ev->buffer_length, ev->data[i]); 
    if (written != ev->buffer_length) 
    {
      return NP_ERR_NOT_ENOUGH_BYTES; 
    }
  }

  written = generic_write(gf, sizeof(ev->board_id), &ev->board_id); 
  if (written != ev->board_id) 
  {
      return NP_ERR_NOT_ENOUGH_BYTES; 
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


  //add additional cases if necessary 
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

      for (i = 0; i < NP_NUM_CHAN; i++)
      {
        wanted = ev->buffer_length; 
        got = generic_read(gf, wanted, ev->data[i]); 
        if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
        cksum = stupid_fletcher16_append(wanted, ev->data[i], cksum); 

        // zero out the rest of the memory 
        memset(ev->data[i] + wanted, 0, NP_MAX_WAVEFORM_LENGTH - wanted); 
      }

      wanted = sizeof(ev->board_id); 
      got = generic_read(gf, wanted, &ev->board_id); 
      if (wanted != got) return NP_ERR_NOT_ENOUGH_BYTES; 
      cksum = stupid_fletcher16_append(wanted, &ev->board_id,cksum); 
  }
  else
  {
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

  got = packet_start_read(gf, &start, NUPHASE_STATUS_MAGIC, NUPHASE_STATUS_VERSION); 
  if (!got) return got; 

  switch(start.ver) 
  {
    //add cases here if necessary 
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



/* pretty prints */ 

int nuphase_print_status(FILE *f, const nuphase_status_t *st)
{
  int i ; 
  struct tm  tim; 
  char timstr[128]; 
  gmtime_r((time_t*) &st->readout_time, &tim); 
  strftime(timstr,sizeof(timstr), "%Y-%m-%d $H:%M:%S", &tim);  
  fprintf(f,"NuPhase Board 0x%x Status (read at %s.%09d UTC)\n", st->board_id, timstr, st->readout_time_ns); 
  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    fprintf(f,"\tBEAM %d:  %u \n",i, st->scalers[i]); 
  }
  return 0; 
}


