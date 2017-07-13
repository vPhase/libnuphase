#include "nuphase.h" 

#include <string.h>

//these need to be incremented if the structs change incompatibly
//and then generic_*_read must be updated to delegate appropriately. 
#define NUPHASE_HEADER_VERSION 0 
#define NUPHASE_EVENT_VERSION 0 

#define NUPHASE_HEADER_MAGIC 0xa1 
#define NUPHASE_EVENT_MAGIC  0xd0 

//TODO there are apparently much faster versions of these 

static uint16_t stupid_fletcher16_append(int N, const void * vbuf, uint16_t append) 

{
  int i; 
  uint16_t sum1 = append  & 0xff; 
  uint16_t sum2 = append >> 8;; 
  uint8_t * buf = (uint8_t*) vbuf; 

  for (i < 0; i < N; i++)
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



/* This just sets defaults */ 
void nuphase_config_init(nuphase_config_t * c) 
{
  c->channel_mask = 0xff; 
  c->pretrigger = 1; //?? 
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
  
  if (written != sizeof(nuphase_header_t), h)
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  return 0; 
}


static int nuphase_header_generic_read(struct generic_file gf, nuphase_header_t *h) 
{
  struct packet_start start; 
  int read; 
  int wanted; 
  uint16_t cksum; 

  read = generic_read(gf, sizeof(start.magic), &start.magic); 

  if (read != sizeof(start.magic)) return NP_ERR_NOT_ENOUGH_BYTES; 


  if (start.magic != NUPHASE_HEADER_MAGIC)
    return NP_ERR_WRONG_TYPE; 

  read = generic_read(gf, sizeof(start.ver), &start.ver); 
  if (read != sizeof(start.ver)) return NP_ERR_NOT_ENOUGH_BYTES; 

  if (start.ver > NUPHASE_HEADER_VERSION) 
    return NP_ERR_BAD_VERSION; 

  read = generic_read(gf,sizeof(start.cksum), &start.cksum); 
  if (read != sizeof(start.cksum)) return NP_ERR_NOT_ENOUGH_BYTES; 

  switch(start.ver) 
  {
    //add cases here if necessary 
    case NUPHASE_HEADER_VERSION: //this is the most recent header!
      wanted = sizeof(nuphase_header_t); 
      read = generic_read(gf, wanted, h); 
      cksum = stupid_fletcher16(wanted, h); 
      break; 
    default: 
    return NP_ERR_BAD_VERSION; 
  }

  if (wanted!=read)
  {
    return NP_ERR_NOT_ENOUGH_BYTES; 
  }

  if (cksum != start.cksum) 
  {
    return NP_ERR_CHECKSUM_FAILED; 
  }

  return 0; 
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

  return 0; 
}


static int nuphase_event_generic_read(struct generic_file gf, nuphase_event_t *ev) 
{
  struct packet_start start; 
  int read; 
  int wanted; 
  uint16_t cksum; 
  int i; 

  read = generic_read(gf, sizeof(start.magic), &start.magic); 

  if (read != sizeof(start.magic)) return NP_ERR_NOT_ENOUGH_BYTES; 


  if (start.magic != NUPHASE_EVENT_MAGIC)
    return NP_ERR_WRONG_TYPE; 

  read = generic_read(gf, sizeof(start.ver), &start.ver); 
  if (read != sizeof(start.ver)) return NP_ERR_NOT_ENOUGH_BYTES; 

  if (start.ver > NUPHASE_EVENT_VERSION) 
    return NP_ERR_BAD_VERSION; 

  read = generic_read(gf,sizeof(start.cksum), &start.cksum); 
  if (read != sizeof(start.cksum)) return NP_ERR_NOT_ENOUGH_BYTES; 


  //add additional cases if necessary 
  if (start.ver == NUPHASE_EVENT_VERSION) 
  {
      wanted = sizeof(ev->event_number); 
      read = generic_read(gf, wanted, &ev->event_number); 
      if (wanted != read) return NP_ERR_NOT_ENOUGH_BYTES; 
      cksum = stupid_fletcher16(wanted, &ev->event_number); 
 
      wanted = sizeof(ev->buffer_length); 
      read = generic_read(gf, wanted, &ev->buffer_length); 

      if (wanted != read) return NP_ERR_NOT_ENOUGH_BYTES; 

      cksum = stupid_fletcher16_append(wanted, &ev->buffer_length,cksum); 

      for (i = 0; i < NP_NUM_CHAN; i++)
      {
        wanted = ev->buffer_length; 
        read = generic_read(gf, wanted, ev->data[i]); 
        if (wanted != read) return NP_ERR_NOT_ENOUGH_BYTES; 
        cksum = stupid_fletcher16_append(wanted, ev->data[i], cksum); 

        // zero out the rest of the memory 
        memset(ev->data[i] + wanted, 0, NP_MAX_WAVEFORM_LENGTH - wanted); 
      }
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




