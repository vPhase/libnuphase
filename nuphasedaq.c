#include "nuphasedaq.h" 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>
#include <pthread.h> 

#define NP_ADDRESS_MAX 128 
#define NP_SPI_BYTES  NP_WORD_SIZE
#define NP_NUM_MODE 4
#define NP_NUM_REGISTER 16

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
  REG_CALPULSE=0x2a, //cal pulse
  REG_READ=0x47, //send data to spi miso 
  REG_FORCE_TRIG=0x40, 
  REG_CHANNEL=0x41, //select channel to read
  REG_MODE=0x42, //readout mode
  REG_RAM_ADDR=0x45, //ram address
  REG_CHUNK=0x49, //which 32-bit chunk 
  REG_BUFFER=0x4e 
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
static uint8_t buf_read[NP_SPI_BYTES] = {REG_READ,0,0,0}; 


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
}



static init_xfers(int n, struct spi_ioc_transfer * xfers)
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
}; 

#define USING(d) if (d->enable_locking) pthread_mutex_lock(&d->mut);
#define DONE(d)  if (d->enable_locking) pthread_mutex_unlock(&d->mut);

static int setup_change_mode(struct spi_ioc_transfer * xfer, nuphase_readout_mode_t mode)
{
  xfer->tx_buf = (uint64_t) buf_mode[mode]; 
}



/* this will use up 3 xfer's, assumed to be zeroed already. DOES NOT SET MODE.  */
static int setup_read_register(struct spi_ioc_transfer * xfers, uint8_t address, uint8_t *result)
{

  xfers[0].tx_buf = (uint64_t)  buf_set_read_reg[address]; 
  xfers[1].tx_buf = (uint64_t)  buf_read; 
  xfers[2].rx_buf = (uint64_t)  result; 

  return 0; 
}


/* this will use up 13*naddr xfers.  MUST start on chunk 0 */ 
static int loop_over_chunks_half_duplex(struct spi_ioc_transfer * xfers, uint8_t naddr, uint8_t start_address, uint8_t * result) 
{

  int iaddr; 
  int ichunk; 
  int ixfer = 0; 

  for (iaddr  =0; iaddr < naddr; iaddr++) 
  {
    xfers[ixfer++].tx_buf = (uint64_t) buf_ram_addr[ start_address + iaddr]; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      xfers[ixfer++].tx_buf = (uint64_t) buf_chunk[ichunk]; 
      xfers[ixfer++].tx_buf = (uint64_t) buf_read; 
      xfers[ixfer++].rx_buf = (uint64_t) ( result + NP_NUM_CHUNK * iaddr + ichunk * NP_SPI_BYTES); 
    }
  }

}

// we can do up to 511 ioctl's at a time. We will use 3 xfers to set the readout mode, buffer and channel 
// that means the maximum number of addresses at a time with half duplex is 39 (156 chunks)
int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start, uint8_t finish, uint8_t * data) 
{
  const int XFERS_PER_ADDR = 3 * NP_NUM_CHUNK + 1 ;
  const int MAX_ADDR = (511-3) / XFERS_PER_ADDR; 
  const int NXFERS = MAX_ADDR * XFERS_PER_ADDR + 3; 

  struct spi_ioc_transfer xfers[NXFERS]; //the maximum number (507 for reads, 1 for setting mode, 1 for setting channel, 1 for setting  buffer) 
  uint8_t naddress = finish - start + 1; 
  uint8_t leftover = naddress % MAX_ADDR; 
  uint8_t niter = naddress / MAX_ADDR + (leftover ? 1 : 0); 
  uint8_t i; 
  init_xfers(NXFERS,xfers); 

  setup_change_mode(xfers, MODE_WAVEFORMS); 
  xfers[1].tx_buf = (uint64_t) buf_buffer[buffer]; 
  xfers[2].tx_buf = (uint64_t) buf_channel[channel]; 
  USING(d);  //have to lock for the duration otherwise channel /read mode may be changed form underneath us
  for (i = 0; i< niter; i++)
  {
    int naddr = i == niter -1 ? leftover : MAX_ADDR; 
    int nxfers = (i == 0 ? 3 : 0)  + XFERS_PER_ADDR * naddr; 
    loop_over_chunks_half_duplex(xfers+3, naddr, start + i * MAX_ADDR, data + i * MAX_ADDR * NP_NUM_CHUNK* NP_SPI_BYTES); 
    ioctl(d->spi_fd, SPI_IOC_MESSAGE(nxfers), (i == 0 ? xfers : xfers+3) );
  }
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



nuphase_dev_t * nuphase_open(const char * devicename, const char * gpio, int locking)
{
  int fd = open(devicename, O_RDWR); 
  nuphase_config_t cfg; 
  if (fd < 0) return 0; 


  nuphase_dev_t * dev = malloc(sizeof(nuphase_dev_t)); 
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


  uint32_t speed = 10000000; //10 MHz 
  uint8_t mode = SPI_MODE_0; 

  ioctl(dev->spi_fd, SPI_IOC_WR_MODE, &mode); 
  ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); 

  //send the default config 
  nuphase_config_init(&cfg); 

  // if this is still running in 20 years, someone will have to fix the y2k38 problem 
  dev->event_number = ((uint64_t)time(0)) << 32; 
  dev->buffer_length = 624; 

  dev->enable_locking = locking; 

  if (locking) 
  {
    pthread_mutex_init(&dev->mut,0); 
  }

  return dev; 
}


void nuphase_set_event_number(nuphase_dev_t * d, uint64_t number)
{
  d->event_number = number; 
}

void nuphase_set_buffer_length(nuphase_dev_t * d, uint16_t length)
{
  d->buffer_length = length; 
}

uint64_t nuphase_get_event_number(const nuphase_dev_t * d) 
{
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

  return ret; 
}


int nuphase_close(nuphase_dev_t * d) 
{
  int ret = 0; 
  USING(d); 

  ret += close(d->spi_fd); 
  d->spi_fd = 0; 
  if (d->gpio_fd)
  {
    ret += 2 * close(d->gpio_fd); 
  }

  if (d->enable_locking)
  {
    pthread_mutex_unlock(&d->mut); 
    ret += 64* pthread_mutex_destroy(&d->mut); 
    d->enable_locking = 0; 
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
  mask  = result &  0xf; // only keep lower 4 bits. who knows what's in the other bits? 
  return mask; 
}



int nuphase_configure(nuphase_dev_t * d, const nuphase_config_t *c) 
{
  //TODO 
}


int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                      nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                      nuphase_event_t  (*events)[NP_NUM_BUFFER])  
{
  nuphase_buffer_mask_t mask; ; 
  mask = nuphase_wait(d); 
  if (mask) 
  {
    int ret; 
    ret = nuphase_read_multiple(d,mask,&(*headers)[0], &(*events)[0]); 
    if (!ret) return __builtin_popcount(mask); 
    else return -1; 
  }
  return 0; 
}

int nuphase_read_single(nuphase_dev_t *d, uint8_t buffer, nuphase_header_t * header, nuphase_event_t * event)
{
  nuphase_buffer_mask_t mask = 1 << buffer; 
  return nuphase_read_multiple(d,mask,header, event); 

}


int nuphase_read_multiple(nuphase_dev_t *d, nuphase_buffer_mask_t mask, nuphase_header_t * headers,  nuphase_event_t * events) 
{

  //TODO
}
