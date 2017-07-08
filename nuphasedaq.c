#include "nuphasedaq.h" 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <linux/spi/spidev.h>

#define NP_SPI_BYTES 4 
#define NP_ADDRESS_MAX 128 
#define NP_NUM_CHAN 8 
#define NP_NUM_MODE 4
#define NP_NUM_REGISTER 16
#define NP_NUM_CHUNK 4 


//register map 
typedef enum
{
  REG_FIRMWARE_VER=0x01, 
  REG_FIRMWARE_DATE=0x02, 
  REG_SET_READ_REG=0x00, 
  REG_READ=0x47, //send data to spi miso 
  REG_FORCE_TRIG=0x40, 
  REG_CHANNEL=0x41, //select channel to read
  REG_CHUNK=0x49, //which 32-bit chunk 
  REG_RAM_ADDR=0x45, //ram address
  REG_MODE=0x42, //readout mode
  REG_CALPULSE=0x2a //cal pulse
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
static uint8_t buf_mode[NP_NUM_MODE][4];
static uint8_t buf_set_read_reg[NP_NUM_REGISTER][4];
static uint8_t buf_channel[NP_NUM_CHAN][4];
static uint8_t buf_chunk[NP_NUM_CHUNK][4];
static uint8_t buf_ram_addr[NP_ADDRESS_MAX][4];
static uint8_t buf_read[4] = {REG_READ,0,0,0}; 


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
  }

}



struct nuphase_dev
{
  const char * device_name; 
  int spi_fd; 
  int gpio_fd; 
}; 


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


/* this will use up 3*nchunks + nchunks/4 xfers.  MUST start on chunk 0 */ 
static int loop_over_chunks_half_duplex(struct spi_ioc_transfer * xfers, uint8_t nchunks, uint8_t start_address, uint8_t * result) 
{

  int ichunk; 
  int ixfer = 0; 

  for (ichunk  =0; ichunk < nchunks; ichunk++) 
  {
    if (ichunk % 4 == 0) 
    {
      xfers[ixfer++].tx_buf = (uint64_t) buf_ram_addr[ start_address + ichunk/4]; 
    }

    xfers[ixfer++].tx_buf = (uint64_t) buf_chunk[ichunk%4]; 
    xfers[ixfer++].tx_buf = (uint64_t) buf_read; 
    xfers[ixfer++].rx_buf = (uint64_t) ( result + ichunk * NP_SPI_BYTES); 
  }
 



}


// we can do up to 511 ioctl's at a time
// that means the maximum number of addresses at a time with half duplex is 39 (156 chunks)
#define MAX_CHUNKS 39 
int nuphase_read_raw(nuphase_dev_t *d, uint8_t channel, uint8_t start, uint8_t finish, uint8_t * data) 
{

  struct spi_ioc_transfer xfers[509]; //the maximum number (507 for chunks, 1 for setting mode, 1 for setting channel) 
  uint8_t naddress = finish - start + 1; 
  uint8_t leftover = naddress % MAX_CHUNKS; 
  uint8_t niter = naddress / MAX_CHUNKS + (leftover ? 1 : 0); 
  uint8_t i; 
  init_xfers(509,xfers); 

  xfers[0].tx_buf = (uint64_t) buf_mode[MODE_WAVEFORMS]; 
  xfers[1].tx_buf = (uint64_t) buf_channel[channel]; 
   

  for (i = 0; i< niter; i++)
  {
    int nchunks = i == niter -1 ? leftover : MAX_CHUNKS; 
    int nxfers = (i == 0 ? 2 : 0)  + 3 * nchunks + nchunks/4; 
    loop_over_chunks_half_duplex(xfers+2, nchunks, start + i * MAX_CHUNKS, data + i * MAX_CHUNKS * NP_SPI_BYTES); 
    ioctl(d->spi_fd, SPI_IOC_MESSAGE(nxfers), (i == 0 ? xfers : xfers+2) );
  }


  return 0; 
}



int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t *result)
{

  struct spi_ioc_transfer xfer[4]; 
  if (address > NP_ADDRESS_MAX) return 0; 
  init_xfers(4,xfer); 
  setup_change_mode(xfer, MODE_REGISTER); 
  setup_read_register(xfer+1, address,result); 
  return ioctl(d->spi_fd, SPI_IOC_MESSAGE(4), xfer); 
}


int nuphase_sw_trigger(nuphase_dev_t * d, unsigned state) 
{
  uint8_t buf[4] = { REG_FORCE_TRIG, 0,0, state & 0xff };  
  return write(d->spi_fd, buf, 4); 
}

int nuphase_calpulse(nuphase_dev_t * d, unsigned state) 
{
  uint8_t buf[4] = { REG_CALPULSE, 0,0, state & 0xff };  
  return write(d->spi_fd, buf, 4); 
}



nuphase_dev_t * nuphase_open(const char * devicename, const char * gpio)
{
  int fd = open(devicename, O_RDWR); 
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


  return dev; 
}


int nuphase_version(nuphase_dev_t * d, nuphase_version_t * version)
{

  //we need to read 5 registers, so 15 xfers 
  struct spi_ioc_transfer xfers[16]; 
  int ret; 

  uint8_t dna_low[4]; 
  uint8_t dna_mid[4]; 
  uint8_t dna_hi[4]; 

  init_xfers(16,xfers); 
  setup_change_mode(xfers, MODE_REGISTER); 
  setup_read_register(xfers+1, REG_FIRMWARE_VER, (uint8_t*) &(version->ver)); 
  setup_read_register(xfers+4, REG_FIRMWARE_DATE, (uint8_t*) &(version->date)); 
  setup_read_register(xfers+7, 4, dna_low); 
  setup_read_register(xfers+10, 5, dna_mid); 
  setup_read_register(xfers+13, 6, dna_hi); 

  ret = ioctl(d->spi_fd, SPI_IOC_MESSAGE(16), xfers); 

  //TODO check this logic. not sure endianness is correct
  uint64_t dna_low_big =  dna_low[0] | dna_low[1] << 8 || dna_low[2] << 8;
  uint64_t dna_mid_big =  dna_mid[0] | dna_mid[1] << 8 || dna_mid[2] << 8;
  uint64_t dna_hi_big =  dna_hi[0] | dna_hi[1] << 8 ;
  version->dna =  (dna_low_big & 0xffffff) | ( (dna_mid_big & 0xffffff) << 24) | ( (dna_hi_big & 0xffff) << 48); 

  return ret; 
}


int nuphase_close(nuphase_dev_t * d) 
{
  int ret = 0; 

  ret += close(d->spi_fd); 
  d->spi_fd = 0; 
  if (d->gpio_fd)
  {
    ret += 2 * close(d->gpio_fd); 
  }

  free(d); 
  return ret; 
}

int nuphase_wait(nuphase_dev_t * d) 
{

  if (!d->gpio_fd) 
  {
    /* we have to poll, but I don't know how yet */ 
    return -1; 
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

  return 0; 
  
}





