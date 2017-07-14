#include "nuphasedaq.h" 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>
#include <pthread.h> 
#include <arpa/inet.h> 

#define NP_ADDRESS_MAX 128 
#define NP_SPI_BYTES  NP_WORD_SIZE
#define NP_NUM_MODE 4
#define NP_NUM_REGISTER 128
#define BUF_MASK 0xf
#define MAX_PRETRIGGER 8 

#define SPI_CAST  (uintptr_t) 

#define NP_DELAY_USECS 0

#define POLL_USLEEP 1000 


//register map 
typedef enum
{
  REG_SET_READ_REG=0x00, 
  REG_FIRMWARE_VER=0x01, 
  REG_FIRMWARE_DATE=0x02, 
  REG_STATUS = 0x03, 
  REG_CHIPID_LOW = 0x04,  
  REG_CHIPID_MID = 0x05,  
  REG_CHIPID_HI = 0x06,  
  REG_SCALER_READ = 0x07, 
  REG_UPDATE_SCALERS = 0x28, 
  REG_PICK_SCALER = 0x29, 
  REG_CALPULSE=0x2a, //cal pulse
  REG_CHANNEL_MASK=0x30, 
  REG_READ=0x47, //send data to spi miso 
  REG_FORCE_TRIG=0x40, 
  REG_CHANNEL=0x41, //select channel to read
  REG_MODE=0x42, //readout mode
  REG_RAM_ADDR=0x45, //ram address
  REG_CHUNK=0x49, //which 32-bit chunk 
  REG_PRETRIGGER=0x4c, 
  REG_CLEAR=0x4d, //clear buffers 
  REG_BUFFER=0x4e,
  REG_TRIGGER_MASK =0x50, 
  REG_THRESHOLDS= 0x56 // add the threshold to this to get the right register

} nuphase_register_t; 


//readout modes 
typedef enum 
{
  MODE_REGISTER=0,
  MODE_WAVEFORMS=1,
  MODE_BEAMS=2,
  MODE_POWERSUM=3
} nuphase_readout_mode_t; 



// all possible buffers we might batch
static uint8_t buf_mode[NP_NUM_MODE][NP_SPI_BYTES];
static uint8_t buf_set_read_reg[NP_NUM_REGISTER][NP_SPI_BYTES];
static uint8_t buf_channel[NP_NUM_CHAN][NP_SPI_BYTES];
static uint8_t buf_buffer[NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_chunk[NP_NUM_CHUNK][NP_SPI_BYTES];
static uint8_t buf_ram_addr[NP_ADDRESS_MAX][NP_SPI_BYTES];
static uint8_t buf_clear[1 << NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_pick_scaler[NP_NUM_BEAMS][NP_SPI_BYTES]; 

static uint8_t buf_read[NP_SPI_BYTES] = {REG_READ,0,0,0}; 
static uint8_t buf_update_scalers[NP_SPI_BYTES] = {REG_UPDATE_SCALERS,0,0,1} ; 

static void fillBuffers() __attribute__((constructor)); //this will fill them


void fillBuffers()
{
  int i; 

  memset(buf_mode,0,sizeof(buf_mode)); 
  for (i = 0; i < NP_NUM_MODE; i++) 
  {
    buf_mode[i][0] = REG_MODE; 
    buf_mode[i][3] = i; 
  }


  memset(buf_set_read_reg,0,sizeof(buf_set_read_reg)); 
  for (i = 0; i < NP_NUM_REGISTER; i++) 
  {
    buf_set_read_reg[i][0]=  REG_SET_READ_REG; 
    buf_set_read_reg[i][3] = i; 
  }

  memset(buf_channel,0,sizeof(buf_channel)); 
  for (i = 0; i < NP_NUM_CHAN; i++) 
  {
    buf_channel[i][0] = REG_CHANNEL; 
    buf_channel[i][3] = i; 
  }

  memset(buf_buffer,0,sizeof(buf_buffer)); 
  for (i = 0; i < NP_NUM_BUFFER; i++) 
  {
    buf_buffer[i][0] = REG_BUFFER; 
    buf_buffer[i][3] = i; 
  }

  memset(buf_ram_addr,0,sizeof(buf_ram_addr)); 
  for (i = 0; i < NP_ADDRESS_MAX; i++) 
  {
    buf_ram_addr[i][0] = REG_RAM_ADDR;
    buf_ram_addr[i][3] = i;
  }

  memset(buf_chunk,0,sizeof(buf_chunk)); 

  for (i = 0; i < NP_NUM_CHUNK;i++)
  {
    buf_chunk[i][0]=REG_CHUNK; 
    buf_chunk[i][3] = i; 
  }

  memset(buf_clear,0,sizeof(buf_clear)); 
  for (i = 0; i < (1 << NP_NUM_BUFFER); i++)
  {
    buf_chunk[i][0]=REG_CLEAR; 
    buf_chunk[i][3]=i;  
  }

  memset(buf_pick_scaler,0,sizeof(buf_pick_scaler)); 
  for (i = 0; i < (1 << NP_NUM_BEAMS); i++)
  {
    buf_chunk[i][0]=REG_PICK_SCALER; 
    buf_chunk[i][3]=i;  
  }
}



static void init_xfers(int n, struct spi_ioc_transfer * xfers)
{
  int i; 

  memset(xfers,0,n *sizeof(struct spi_ioc_transfer)); 
  for (i = 0; i < n; i++)
  {
    xfers[i].len = NP_SPI_BYTES; 
    xfers[i].cs_change =1; //deactivate cs between transfers
    xfers[i].delay_usecs = NP_DELAY_USECS; //? 
  }
}



struct nuphase_dev
{
  const char * device_name; 
  int spi_fd; 
  int gpio_fd; 
  int enable_locking; 
  uint64_t event_number; 
  nuphase_config_t cfg; 
  uint16_t buffer_length; 
  pthread_mutex_t mut; //mutex for the SPI (not for the gpio though). Only used if enable_locking is true
  uint8_t board_id; 
}; 

#define USING(d) if (d->enable_locking) pthread_mutex_lock(&d->mut);
#define DONE(d)  if (d->enable_locking) pthread_mutex_unlock(&d->mut);

static int setup_change_mode(struct spi_ioc_transfer * xfer, nuphase_readout_mode_t mode)
{
  xfer->tx_buf = SPI_CAST  buf_mode[mode]; 
  return 0; 
}



/* this will use up 3 xfer's, assumed to be zeroed already. DOES NOT SET MODE.  */
static int setup_read_register(struct spi_ioc_transfer * xfers, uint8_t address, uint8_t *result)
{

  xfers[0].tx_buf = SPI_CAST  buf_set_read_reg[address]; 
  xfers[1].tx_buf = SPI_CAST  buf_read; 
  xfers[2].rx_buf = SPI_CAST  result; 

  return 0; 
}

/* to simplify batching complicated transfers,
 * we use this simple data structure. 
 *
 */ 
struct xfer_buffer 
{
  struct spi_ioc_transfer spi[511]; 
  int nused; 
  int fd; 
};

static void xfer_buffer_init(struct xfer_buffer * b, int fd) 
{
  init_xfers(511,b->spi); 
  b->nused = 0; 
  b->fd = fd; 
}


static int xfer_buffer_send(struct xfer_buffer *b)
{
  int success = ioctl(b->fd, SPI_IOC_MESSAGE(b->nused), b->spi); 
  if (!success) 
  {
    fprintf(stderr,"IOCTL failed!\n"); 
    return -1; 
  }
  b->nused = 0; 

  return 0; 
}

// this will send if full!  
static int xfer_buffer_append(struct xfer_buffer * b, const uint8_t * txbuf, const uint8_t * rxbuf) 
{
  //check if full 
  if (b->nused == 511) 
  {
    if (xfer_buffer_send(b))
    {
      return -1; 
    }
  }

  b->spi[b->nused].tx_buf = SPI_CAST txbuf; 
  b->spi[b->nused].rx_buf = SPI_CAST rxbuf; 
  b->nused++; 
  return 0; 
}


/* this will use up 13*naddr xfers.  MUST start on chunk 0 */ 
static int loop_over_chunks_half_duplex(struct xfer_buffer * xfers, uint8_t naddr, uint8_t start_address, uint8_t * result) 
{

  int iaddr; 
  int ichunk; 
  int ret = 0; 

  for (iaddr  =0; iaddr < naddr; iaddr++) 
  {
    ret += xfer_buffer_append(xfers, buf_ram_addr[start_address + iaddr], 0); 
    if (ret) return ret; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      ret+= xfer_buffer_append(xfers, buf_chunk[ichunk], 0); 
      if (ret) return ret; 

      ret+= xfer_buffer_append(xfers, buf_read, 0); 
      if (ret) return ret; 

      ret+= xfer_buffer_append(xfers, 0, result + NP_NUM_CHUNK * iaddr + ichunk * NP_SPI_BYTES); 
      if (ret) return ret; 
    }
  }

  return 0; 
}

int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start, uint8_t finish, uint8_t * data) 
{

  struct xfer_buffer xfers; 
  xfer_buffer_init(&xfers, d->spi_fd); 
  uint8_t naddress = finish - start + 1; 
  int ret = 0; 
  ret += xfer_buffer_append(&xfers, buf_mode[MODE_WAVEFORMS], 0);  if (ret) return 0; 
  ret += xfer_buffer_append(&xfers, buf_buffer[buffer], 0);  if (ret) return 0; 
  ret += xfer_buffer_append(&xfers, buf_channel[channel], 0);  if (ret) return 0; 
  USING(d);  //have to lock for the duration otherwise channel /read mode may be changed form underneath us.  
            // we don't lock before these because there is no way we sent enough transfers to trigger a read 
  ret += loop_over_chunks_half_duplex(&xfers, naddress, start, data);
  if(!ret) xfer_buffer_send(&xfers); //pick up the stragglers. 
  DONE(d);  

  return 0; 
}



int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t *result)
{

  struct spi_ioc_transfer xfer[4]; 
  int ret;
  if (address > NP_ADDRESS_MAX) return -1; 
  init_xfers(4,xfer); 
  setup_change_mode(xfer, MODE_REGISTER); 
  setup_read_register(xfer+1, address,result); 
  USING(d); 
  ret = ioctl(d->spi_fd, SPI_IOC_MESSAGE(4), xfer); 
  DONE(d);
  return ret; 
}


int nuphase_sw_trigger(nuphase_dev_t * d ) 
{
  uint8_t buf[4] = { REG_FORCE_TRIG, 0,0, 1};  
  int ret; 
  USING(d); 
  ret = write(d->spi_fd, buf, NP_SPI_BYTES); 
  DONE(d); 
  return ret == NP_SPI_BYTES ? 0 : -1;  
}

int nuphase_calpulse(nuphase_dev_t * d, unsigned state) 
{
  uint8_t buf[4] = { REG_CALPULSE, 0,0, state & 0xff };  
  int ret;
  USING(d); 
  ret = write(d->spi_fd, buf, NP_SPI_BYTES); 
  DONE(d); 
  return ret == NP_SPI_BYTES ? 0 : 1 ;
}


static int board_id_counter =1; 

nuphase_dev_t * nuphase_open(const char * devicename, const char * gpio,
                             const nuphase_config_t * c,  int locking)
{
  int locked,fd; 
  nuphase_dev_t * dev; 


  fd = open(devicename, O_RDWR); 
  if (fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", devicename); 
    return 0; 
  }

  locked = flock(fd,LOCK_EX | LOCK_NB); 
  if (locked < 0) 
  {
    fprintf(stderr,"Could not get exclusive access to %s\n", devicename); 
    return 0; 
  }

  dev = malloc(sizeof(nuphase_dev_t)); 
  dev->device_name = devicename; 
  dev->spi_fd =fd;

  if (gpio) 
  {
    dev->gpio_fd = open(gpio, O_RDWR); 
    if (dev->gpio_fd < 0) dev->gpio_fd = 0; 
    else
    {
      uint32_t unmask = 1; //unmask the interrupt 
      write(dev->gpio_fd,&unmask,sizeof(unmask)); 
    }
  }
  else
  {
    dev->gpio_fd = 0; 
  }


  //Configure the SPI protocol 
  uint32_t speed = 10000000; //10 MHz 
  uint8_t mode = SPI_MODE_0; 
  ioctl(dev->spi_fd, SPI_IOC_WR_MODE, &mode); 
  ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); 


  //configuration 
  if (c) dev->cfg = *c; 
  else nuphase_config_init(&dev->cfg); 
  nuphase_configure(dev, &dev->cfg); 

  // if this is still running in 20 years, someone will have to fix the y2k38 problem 
  dev->event_number = ((uint64_t)time(0)) << 32; 
  dev->buffer_length = 624; 
  dev->board_id = board_id_counter++; 

  dev->enable_locking = locking; 

  if (locking) 
  {
    pthread_mutex_init(&dev->mut,0); 
  }

  return dev; 
}

void nuphase_set_board_id(nuphase_dev_t * d, uint8_t id)
{
  d->board_id = id; 
}

uint8_t nuphase_get_board_id(const nuphase_dev_t * d) 
{
  return d->board_id; 
}


void nuphase_set_event_number(nuphase_dev_t * d, uint64_t number)
{
  d->event_number = number; 
  //TODO this will eventually set it on the board 
}

void nuphase_set_buffer_length(nuphase_dev_t * d, uint16_t length)
{
  d->buffer_length = length; 
}

uint64_t nuphase_get_event_number(const nuphase_dev_t * d) 
{
  // this will be checked with the board on every read
  return d->event_number; 
}

uint16_t nuphase_get_bufferlength(const nuphase_dev_t * d) 
{
  return d->buffer_length; 
}


int nuphase_fwinfo(nuphase_dev_t * d, nuphase_fwinfo_t * info)
{

  //we need to read 5 registers, so 15 xfers 
  struct spi_ioc_transfer xfers[16]; 
  int ret; 

  uint8_t dna_low[4]; 
  uint8_t dna_mid[4]; 
  uint8_t dna_hi[4]; 

  init_xfers(16,xfers); 
  setup_change_mode(xfers, MODE_REGISTER); 
  setup_read_register(xfers+1, REG_FIRMWARE_VER, (uint8_t*) &(info->ver)); 
  setup_read_register(xfers+4, REG_FIRMWARE_DATE, (uint8_t*) &(info->date)); 
  setup_read_register(xfers+7, REG_CHIPID_LOW, dna_low); 
  setup_read_register(xfers+10, REG_CHIPID_MID, dna_mid); 
  setup_read_register(xfers+13, REG_CHIPID_HI, dna_hi); 


  USING(d); 
  ret = ioctl(d->spi_fd, SPI_IOC_MESSAGE(16), xfers); 
  DONE(d); 


  //TODO check this logic. not sure endianness is correct
  uint64_t dna_low_big =  dna_low[0] | dna_low[1] << 8 || dna_low[2] << 8;
  uint64_t dna_mid_big =  dna_mid[0] | dna_mid[1] << 8 || dna_mid[2] << 8;
  uint64_t dna_hi_big =  dna_hi[0] | dna_hi[1] << 8 ;
  info->dna =  (dna_low_big & 0xffffff) | ( (dna_mid_big & 0xffffff) << 24) | ( (dna_hi_big & 0xffff) << 48); 

  //ver and date are going to be the wrong endianness, I think 
  info->ver = ntohl(info->ver); 
  info->date = ntohl(info->date); 

  return ret; 
}


int nuphase_close(nuphase_dev_t * d) 
{
  int ret = 0; 
  USING(d); 
  if (d->enable_locking)
  {
    //this should be allowed? 
    pthread_mutex_unlock(&d->mut); 
    ret += 64* pthread_mutex_destroy(&d->mut); 
    d->enable_locking = 0; 
  }

  ret += flock(d->spi_fd, LOCK_UN); 
  ret += 4*close(d->spi_fd); 
  if (d->gpio_fd)
  {
    ret += 8 * close(d->gpio_fd); 
  }
  free(d); 
  return ret; 
}

nuphase_buffer_mask_t nuphase_wait(nuphase_dev_t * d) 
{

  if (!d->gpio_fd) 
  {
    nuphase_buffer_mask_t something = 0; 
    while(!something)
    {
      usleep(POLL_USLEEP); 
      something = nuphase_check_buffers(d); 
    }
    return something; 
  }

  uint32_t info; 
  struct pollfd fds = { .fd = d->gpio_fd, .events = POLLIN }; 
  int ret = poll(&fds,1,-1);  
  if (ret < 0) return ret; 

  int nb = read(d->gpio_fd,&info, sizeof(info)); 

  if (nb  == sizeof(info)) 
  {
    //let's unmask the interrupt
    uint32_t unmask = 1;
    if (write(d->gpio_fd,&unmask,sizeof(unmask)) < 0) 
    {
      return -1; 
    }
  }

 return  nuphase_check_buffers(d); 
}


nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t * d) 
{

  uint8_t result; 
  nuphase_buffer_mask_t mask; 
  nuphase_read_register(d, REG_STATUS, &result); 
  mask  = result &  BUF_MASK; // only keep lower 4 bits. who knows what's in the other bits? 
  return mask; 
}

/* This just sets defaults */ 
void nuphase_config_init(nuphase_config_t * c) 
{
  int i;
  c->channel_mask = 0xff; 
  c->pretrigger = 1; //?? 
  c->trigger_mask = 0xfff; 

  for (i = 0; i < NP_NUM_BEAMS; i++)
    c->trigger_thresholds[i] = 0xfffff;  //TODO make this more sensible by default 
}


int nuphase_configure(nuphase_dev_t * d, const nuphase_config_t *c) 
{
  int written; 
  int ret = 0; 
  
  USING(d); 
  if (c->pretrigger != d->cfg.pretrigger)
  {
    uint8_t pretrigger_buf[] = { REG_PRETRIGGER, 0, 0, c->pretrigger}; 
    written = write(d->spi_fd, pretrigger_buf, NP_SPI_BYTES); 
    if (written == NP_SPI_BYTES) 
    {
      d->cfg.pretrigger = c->pretrigger; 
    }
    else
    {
      ret = -1; //don't try to continue if we fail . 
      goto configure_end; 
    }
  }

  if (c->channel_mask!= d->cfg.channel_mask)
  {
    uint8_t channel_mask_buf[]= { REG_PRETRIGGER, 0, 0, c->channel_mask}; 
    written = write(d->spi_fd, channel_mask_buf, NP_SPI_BYTES); 
    if (written == NP_SPI_BYTES) 
    {
      d->cfg.channel_mask = c->channel_mask; 
    }
    else
    {
      ret = -1; 
      goto configure_end; 
    }
  }

  if (c->trigger_mask != d->cfg.trigger_mask)
  {
    //TODO check the byte order
    uint8_t trigger_mask_buf[]= { REG_TRIGGER_MASK, 0, (c->trigger_mask >> 8) & 0xff, c->trigger_mask & 0xff}; 
    written = write(d->spi_fd, trigger_mask_buf, NP_SPI_BYTES); 
    if (written == NP_SPI_BYTES) 
    {
      d->cfg.trigger_mask = c->trigger_mask; 
    }
    else
    {
      ret = -1; 
      goto configure_end; 
    }
  }

  if (memcmp(c->trigger_thresholds, d->cfg.trigger_thresholds, sizeof(c->trigger_thresholds)))
  {
    uint8_t thresholds_buf[NP_NUM_BEAMS][NP_SPI_BYTES]; 
    struct spi_ioc_transfer xfer[NP_NUM_BEAMS]; 
    init_xfers(NP_NUM_BEAMS, xfer); 
    int i; 
    for (i = 0; i < NP_NUM_BEAMS; i++)
    {
      thresholds_buf[i][0]= REG_THRESHOLDS+i ;
      thresholds_buf[i][1]= (c->trigger_thresholds[i] >> 16 ) & 0xf;
      thresholds_buf[i][2]= ( c->trigger_thresholds[i] >> 8) & 0xff; 
      thresholds_buf[i][3]= c->trigger_thresholds[i] & 0xff;
      xfer[i].tx_buf =  SPI_CAST &thresholds_buf[i][0]; 
    }
    if(!ioctl(d->spi_fd, SPI_IOC_MESSAGE(NP_NUM_BEAMS), xfer))
    {
      //success! 
      memcpy(d->cfg.trigger_thresholds, c->trigger_thresholds, sizeof(c->trigger_thresholds)); 
    }
    else 
    {
      ret =-1; 
      goto configure_end; 
    }
  }

  configure_end: 

  DONE(d); 
  return ret; 
}



//indirection! 
int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                      nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                      nuphase_event_t  (*events)[NP_NUM_BUFFER])  
{
  nuphase_buffer_mask_t mask; ; 
  mask = nuphase_wait(d); 
  if (mask) 
  {
    int ret; 
    ret = nuphase_read_multiple_array(d,mask,&(*headers)[0], &(*events)[0]); 
    if (!ret) return __builtin_popcount(mask); 
    else return -1; 
  }
  return 0; 
}

//yay more indirection!
int nuphase_read_single(nuphase_dev_t *d, uint8_t buffer, nuphase_header_t * header, nuphase_event_t * event)
{
  nuphase_buffer_mask_t mask = 1 << buffer; 
  return nuphase_read_multiple_ptr(d,mask,&header, &event); 

}

//woohoo, even more indirection. 
int nuphase_read_multiple_array(nuphase_dev_t *d, nuphase_buffer_mask_t mask, nuphase_header_t * headers,  nuphase_event_t * events) 
{
  nuphase_event_t * ev_ptr_array[NP_NUM_BUFFER]; 
  nuphase_header_t * hd_ptr_array[NP_NUM_BUFFER]; 
  int i; 

  for ( i = 0; i < __builtin_popcount(mask); i++)
  {
    ev_ptr_array[i] = &events[i]; 
    hd_ptr_array[i] = &headers[i]; 
  }

  return nuphase_read_multiple_ptr(d,mask,hd_ptr_array, ev_ptr_array); 
}


int nuphase_read_multiple_ptr(nuphase_dev_t * d, nuphase_buffer_mask_t mask, nuphase_header_t ** hd, nuphase_event_t ** ev)
{
  int ibuf,ichan;
  int iout = 0; 
  int ret; 
  struct xfer_buffer xfers; 
  struct timespec now; 
  clock_gettime(CLOCK_REALTIME, &now); 
  xfer_buffer_init(&xfers, d->spi_fd); 
  USING(d); //hold the lock for the duration
  for (ibuf = 0; ibuf < NP_NUM_BUFFER; ibuf++)
  {
    if ( (mask & (1 << ibuf)) == 0)
      continue; 

    //TODO should probably add some error checks and stuff... 
    //set the buffer 
    ret+=xfer_buffer_append(&xfers, buf_buffer[ibuf],0); 
    if (ret) goto the_end; 

    //TODO read metadata once that is fully implemented 
    
    hd[ibuf]->buffer_mask = mask; 
    hd[ibuf]->buffer_length = d->buffer_length; 
    hd[ibuf]->readout_time = now.tv_sec; 
    hd[ibuf]->readout_time_ns = now.tv_nsec; 
    hd[ibuf]->event_number = d->event_number; //this will be checked against hardware number! 
    hd[ibuf]->board_id = d->board_id; 
    ev[ibuf]->event_number = d->event_number; 
    ev[ibuf]->buffer_length = d->buffer_length; 
    ev[ibuf]->board_id = d->board_id; 

    ret+=xfer_buffer_append(&xfers, buf_mode[MODE_WAVEFORMS],0); 
    if (ret) goto the_end; 
    for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
    {
      ret+=xfer_buffer_append(&xfers, buf_channel[ichan],0); 
      if (ret) goto the_end; 

      ret+=loop_over_chunks_half_duplex(&xfers, hd[ibuf]->buffer_length / (NP_SPI_BYTES * NP_NUM_CHUNK),0, ev[ibuf]->data[ichan]); 
    }
    ret+=xfer_buffer_append(&xfers, buf_clear[1 << ibuf],0); 
    if (ret) goto the_end; 

    ret+=xfer_buffer_send(&xfers); //flush so we can clear the buffer immediately 
    if (ret) goto the_end; 

    iout++; 
  }

  the_end:

  DONE(d); 

  return ret; 
}


int nuphase_clear_buffer(nuphase_dev_t *d, nuphase_buffer_mask_t mask) 
{
  int ret; 
  USING(d); 
  ret = write(d->spi_fd, buf_clear[mask & BUF_MASK], NP_SPI_BYTES); //TODO check if this is really the right interface 
  DONE(d); 
  return ret == NP_SPI_BYTES ? 0 : -1; 
}

int nuphase_write(nuphase_dev_t *d, const uint8_t* buffer)
{
  int written = 0; 
  USING(d); 
  written = write(d->spi_fd, buffer,NP_SPI_BYTES); 
  DONE(d); 
  return written == NP_SPI_BYTES ? 0 : -1; 
}

int nuphase_read(nuphase_dev_t *d,uint8_t* buffer)
{
  int got = 0; 
  USING(d); 
  got = read(d->spi_fd, buffer,NP_SPI_BYTES); 
  DONE(d); 
  return got == NP_SPI_BYTES ? 0 : -1; 
}



int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * st) 
{
  //TODO: fill in deadtime when I figure out how. 
  int ret; 
  int ixfer = 0; 
  int i; 
  struct timespec now; 
  uint8_t wide_scalers[NP_NUM_BEAMS][NP_SPI_BYTES]; 

  st->board_id = d->board_id; 

  //cheating  because const int is apparently not constant in C
  enum {nxfers = 1 + 1 + NP_NUM_BEAMS * 3};   //one set mode register, one update scalers, num_beams *  ( pick scaler, read register, read) 

  _Static_assert (nxfers < 512, "TOO MANY IOC MESSAGES" ); 

  struct spi_ioc_transfer xfers[nxfers];
  init_xfers(nxfers, xfers); 

  xfers[ixfer++].tx_buf = SPI_CAST buf_mode[MODE_REGISTER]; 
  xfers[ixfer++].tx_buf = SPI_CAST buf_update_scalers; 

  for (i = 0; i < NP_NUM_BEAMS; i++) 
  {
    xfers[ixfer++].tx_buf = SPI_CAST buf_pick_scaler[i]; 
    xfers[ixfer++].tx_buf = SPI_CAST buf_set_read_reg[REG_SCALER_READ]; 
    xfers[ixfer++].rx_buf = SPI_CAST wide_scalers[i]; 
  }

  clock_gettime(CLOCK_REALTIME, &now); 
  USING(d); 
  ret = ioctl(d->spi_fd, SPI_IOC_MESSAGE(nxfers), xfers); 
  DONE(d); 

  if (!ret) return ret; 

  st->deadtime = 0; //TODO 
  for (i = 0; i < NP_NUM_BEAMS; i++) 
  {
    st->scalers[i] = wide_scalers[i][3] | (wide_scalers[i][2] << 8 ); 
  }
  st->readout_time = now.tv_sec; 
  st->readout_time_ns = now.tv_nsec; 

  return 0; 
}


