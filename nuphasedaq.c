#define _GNU_SOURCE //required for ppoll. 

#include "nuphasedaq.h" 
#include <linux/spi/spidev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h> 
#include <signal.h>
#include <inttypes.h>
#include <errno.h> 
#include <endian.h>

#define NP_ADDRESS_MAX 128 
#define NP_SPI_BYTES  NP_WORD_SIZE
#define NP_NUM_MODE 4
#define NP_NUM_REGISTER 128
#define BUF_MASK 0xf
#define MAX_PRETRIGGER 8 
#define BOARD_CLOCK_HZ 25000000

#define MIN_GOOD_MAX_V 20 
#define MAX_MISERY 25 

#define SPI_CAST  (uintptr_t) 

#define NP_DELAY_USECS 0
#define NP_CS_CHANGE 1

#define POLL_USLEEP 500
#define SPI_CLOCK 10000000
//#define SPI_CLOCK 1000000

#define MAX_XFERS 511

//#define DEBUG_PRINTOUTS 1 

//register map 
typedef enum
{
  REG_FIRMWARE_VER       = 0x01, 
  REG_FIRMWARE_DATE      = 0x02, 
  REG_SCALER_READ        = 0x03, 
  REG_CHIPID_LOW         = 0x04,  
  REG_CHIPID_MID         = 0x05,  
  REG_CHIPID_HI          = 0x06,  
  REG_STATUS             = 0x07, 
  REG_CLEAR_STATUS       = 0x09, 
  REG_EVENT_COUNTER_LOW  = 0xa, 
  REG_EVENT_COUNTER_HIGH = 0xb, 
  REG_TRIG_COUNTER_LOW   = 0xc, 
  REG_TRIG_COUNTER_HIGH  = 0xd, 
  REG_TRIG_TIME_LOW      = 0xe, 
  REG_TRIG_TIME_HIGH     = 0xf, 
  REG_DEADTIME           = 0x10, 
  REG_TRIG_INFO          = 0x11, //bits 23-22 : event buffer ; bit 21: calpulse, bits 19-17: pretrig window,  bits16-15: trig type ; bits 14-0: last beam trigger
  REG_TRIG_MASKS         = 0x12, // bits 22-15 : channel mask ; bits 14-0 : beam mask
  REG_BEAM_POWER         = 0x14, // add beam to get right register
  REG_CHUNK              = 0x23, //which 32-bit chunk  + i 
  REG_SYNC               = 0x27, 
  REG_UPDATE_SCALERS     = 0x28, 
  REG_PICK_SCALER        = 0x29, 
  REG_CALPULSE           = 0x2a, //cal pulse
  REG_CHANNEL_MASK       = 0x30, 
  REG_ATTEN_012          = 0x32, 
  REG_ATTEN_345          = 0x33, 
  REG_ATTEN_67           = 0x34, 
  REG_ATTEN_APPLY        = 0x35, 
  REG_ADC_CLK_RST        = 0x37,  
  REG_ADC_DELAYS         = 0x38, //add buffer number to get all 
  REG_FORCE_TRIG         = 0x40, 
  REG_CHANNEL            = 0x41, //select channel to read
  REG_MODE               = 0x42, //readout mode
  REG_RAM_ADDR           = 0x45, //ram address
  REG_READ               = 0x47         , //send data to spi miso 
  REG_PRETRIGGER         = 0x4c, 
  REG_CLEAR              = 0x4d, //clear buffers 
  REG_BUFFER             = 0x4e,
  REG_TRIGGER_MASK       = 0x50, 
  REG_TRIG_HOLDOFF       = 0x51, 
  REG_TRIG_ENABLE        = 0x52, 
  REG_THRESHOLDS         = 0x56, // add the threshold to this to get the right register
  REG_SET_READ_REG       = 0x6d, 
  REG_RESET_COUNTER      = 0x7e, 
  REG_RESET_ALL          = 0x7f 

} nuphase_register_t; 


void easy_break_point()
{
  //this is here just for an easy break point
  fprintf(stderr,"OOPS\n"); 
}


//readout modes 
typedef enum 
{
  MODE_REGISTER=0,
  MODE_WAVEFORMS=1,
  MODE_BEAMS=2,
  MODE_POWERSUM=3
} nuphase_readout_mode_t; 

/* For now, allow only one master and slave */ 
static nuphase_dev_t * the_master = 0; 
static nuphase_dev_t * the_slave = 0; 

struct nuphase_dev
{
  const char * device_name; 
  int spi_fd; 
  int gpio_fd; 
  int enable_locking; 
  uint64_t readout_number_offset; 
  uint64_t event_counter;  // should match device...we'll keep this to complain if it doesn't
  nuphase_config_t cfg; 
  uint16_t buffer_length; 
  pthread_mutex_t mut; //mutex for the SPI (not for the gpio though). Only used if enable_locking is true
  pthread_mutex_t wait_mut; //mutex for the waiting. Only used if enable_locking is true
  uint8_t board_id; 
  uint8_t channel_read_mask;// read mask... right now it's always 0xf, but we can make it configurable later
  volatile int cancel_wait; // needed for signal handlers 
  struct timespec start_time; //the time of the last clock reset
  long waiting_thread;     // needed for signal handlers (if gpio is used) 
  uint8_t next_read_buffer; //what buffer to read next 
  uint8_t hardware_next; // what buffer the hardware things we should read next 
  int spi_clock; 
  int cs_change; 
  int delay_us; 

  // store event / header used for calibration here in case we want it later? 
  nuphase_event_t calib_ev; 
  nuphase_header_t calib_hd; 

  //spi buffer 
  struct spi_ioc_transfer buf[MAX_XFERS]; 
  int nused; 

  // device state 
  int current_buf; 
  int current_mode; 

  // for synchronization
  uint8_t sync_buf_clear_mask;  // masks ready to clear on this device 
  
}; 

//some macros 
#define USING(d) if (d->enable_locking) pthread_mutex_lock(&d->mut);
#define DONE(d)  if (d->enable_locking) pthread_mutex_unlock(&d->mut);


//Wrappers for io functions to add printouts 
static int do_xfer(int fd, int n, struct spi_ioc_transfer * xfer) 
{
#ifdef DEBUG_PRINTOUTS
  struct timespec start; 
  struct timespec end; 
  clock_gettime(CLOCK_REALTIME, &start) ; 
#endif
  int ret = ioctl(fd,SPI_IOC_MESSAGE(n), xfer); 

#ifdef DEBUG_PRINTOUTS
  clock_gettime(CLOCK_REALTIME, &end) ; 
  int i; 
  printf("START BULK TRANSFER (t = %lu.%lu)\n", start.tv_sec, start.tv_nsec); 
  for (i = 0; i < n; i++)
  {
      printf("\tXFR %03d\t",i); 
      if (xfer[i].tx_buf)
      {
        uint8_t * tx = (uint8_t*) SPI_CAST xfer[i].tx_buf; 
        printf("TX [0x%02x 0x%02x 0x%02x 0x%02x]\t", tx[0],tx[1],tx[2],tx[3]); 
      }
      if (xfer[i].rx_buf) 
      {
        uint8_t * rx = (uint8_t*) SPI_CAST xfer[i].rx_buf; 
        printf("RX [0x%02x 0x%02x 0x%02x 0x%02x]\t", rx[0],rx[1],rx[2],rx[3]); 
      }
      printf("\n"); 
  }
  printf("END BULK TRANSFER (t= %lu.%lu)\n", end.tv_sec, end.tv_nsec); 
#endif 
  return ret; 
}

static int do_write(int fd, const uint8_t * p)
{
  int ret = write(fd,p, NP_SPI_BYTES); 
#ifdef DEBUG_PRINTOUTS
  printf("WRITE: [0x%02x 0x%02x 0x%02x 0x%02x]\n", p[0],p[1],p[2],p[3]); 
#endif
  return ret; 
}

static int do_read(int fd, uint8_t * p)
{
  int ret = read(fd,p, NP_SPI_BYTES); 
#ifdef DEBUG_PRINTOUTS
  printf("READ: [0x%02x 0x%02x 0x%02x 0x%02x]\n", p[0],p[1],p[2],p[3]); 
#endif
  return ret; 
}



// all possible buffers we might batch
static uint8_t buf_mode[NP_NUM_MODE][NP_SPI_BYTES];
static uint8_t buf_set_read_reg[NP_NUM_REGISTER][NP_SPI_BYTES];
static uint8_t buf_channel[NP_NUM_CHAN][NP_SPI_BYTES];
static uint8_t buf_buffer[NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_chunk[NP_NUM_CHUNK][NP_SPI_BYTES];
static uint8_t buf_ram_addr[NP_ADDRESS_MAX][NP_SPI_BYTES];
static uint8_t buf_clear[1 << NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_pick_scaler[NP_NUM_BEAMS][NP_SPI_BYTES]; 

static uint8_t buf_read[NP_SPI_BYTES] __attribute__((unused))= {REG_READ,0,0,0}  ; 

static uint8_t buf_update_scalers[NP_SPI_BYTES] = {REG_UPDATE_SCALERS,0,0,1} ; 
static uint8_t buf_sync_on[NP_SPI_BYTES] = {REG_SYNC,0,0,1} ; 
static uint8_t buf_sync_off[NP_SPI_BYTES] = {REG_SYNC,0,0,0} ; 
static uint8_t buf_reset_all[NP_SPI_BYTES] = {REG_RESET_ALL,0,0,1}; 
static uint8_t buf_reset_almost_all[NP_SPI_BYTES] = {REG_RESET_ALL,0,0,2}; 
static uint8_t buf_reset_adc[NP_SPI_BYTES] = {REG_RESET_ALL,0,0,4}; 
static uint8_t buf_reset_counter[NP_SPI_BYTES] = {REG_RESET_COUNTER,0,0,1}; 
static uint8_t buf_clear_all_masks[NP_SPI_BYTES] = {REG_TRIGGER_MASK,0,0,0x0}; 
static uint8_t buf_adc_clk_rst[NP_SPI_BYTES] = {REG_ADC_CLK_RST,0,0,0}; 
static uint8_t buf_apply_attenuator[NP_SPI_BYTES] = {REG_ATTEN_APPLY,0,0,0}; 

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
    buf_channel[i][3] = 1<<i; 
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
    buf_chunk[i][0]=REG_CHUNK+i; 
  }

  memset(buf_clear,0,sizeof(buf_clear)); 
  for (i = 0; i < (1 << NP_NUM_BUFFER); i++)
  {
    buf_clear[i][0]=REG_CLEAR; 
    buf_clear[i][3]=i;  
  }

  memset(buf_pick_scaler,0,sizeof(buf_pick_scaler)); 
  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    buf_pick_scaler[i][0]=REG_PICK_SCALER; 
    buf_pick_scaler[i][3]=i;  
  }
}



static void setup_xfers( nuphase_dev_t *d)
{
  int i; 
  for (i = 0; i < MAX_XFERS; i++)
  {
    d->buf[i].len = NP_SPI_BYTES; 
    d->buf[i].cs_change =d->cs_change; //deactivate cs between transfers
    d->buf[i].delay_usecs = d->delay_us;//? 
  }
}


static int buffer_send(nuphase_dev_t * d)
{
  int wrote; 
  if (!d->nused) return 0; 
  wrote = do_xfer(d->spi_fd, d->nused, d->buf); 
  if (wrote < d->nused * NP_SPI_BYTES) 
  {
    fprintf(stderr,"IOCTL failed! returned: %d\n",wrote); 
    return -1; 
  }
  d->nused = 0; 

  return 0; 
}



// this will send if full!  
static int buffer_append(nuphase_dev_t * d, const uint8_t * txbuf, const uint8_t * rxbuf) 
{
  //check if full 
  if (d->nused >= MAX_XFERS) //greater than just in case, but it already means something went horribly wrong 
  {
    if (buffer_send(d))
    {
      return -1; 
    }
  }

  d->buf[d->nused].tx_buf = SPI_CAST txbuf; 
  d->buf[d->nused].rx_buf = SPI_CAST rxbuf; 
  d->nused++; 
  return 0; 
}

static int append_read_register(nuphase_dev_t *d, uint8_t address, uint8_t * result)
{
  int ret = 0; 
  ret += buffer_append(d, buf_set_read_reg[address],0);
  ret += buffer_append(d,0,result); 
  return ret; 
}

static int can_synchronize()
{
  return the_master && the_slave;
}



/* internal synchronized command if reg_to_read_after is not zero, will read a
 * register after (for example to see if something worked) and store the result
 * in the appropriate place. 
 **/ 

static int synchronized_command(const uint8_t * cmd, uint8_t reg_to_read_after,
                                  uint8_t * result_master, uint8_t * result_slave) {
  
  if (!can_synchronize()) 
  {
    fprintf(stderr,"WARNING, tried a synchonized command, but not properly set up (%p %p)\n", 
        the_master, the_slave); 

    return -1; 
  }

  USING(the_master); 
  USING(the_slave); 
  int ret = 0;
  ret+=buffer_append(the_master, buf_sync_on,0); 
  ret+=buffer_append(the_master, cmd,0); 
  ret+=buffer_send(the_master); 
  ret+=buffer_append(the_slave, cmd,0); 
  ret+=buffer_send(the_slave); 
  ret+=buffer_append(the_master, buf_sync_off,0); 
  if (reg_to_read_after) 
  {
    ret+=append_read_register(the_master, reg_to_read_after, result_master); 
  }
  ret+=buffer_send(the_master); 

  if (reg_to_read_after) 
  {
    ret+=append_read_register(the_slave, reg_to_read_after, result_slave); 
    ret+=buffer_send(the_slave); 
  }

  DONE(the_slave); 
  DONE(the_master); 
  return ret; 
}

static int mark_buffer_done(nuphase_dev_t * d,  int ibuf)
{

  if (d == the_master && can_synchronize()) 
  {
    //atomic operation
    __sync_fetch_and_or(&the_master->sync_buf_clear_mask, 1 << ibuf); 
  }
  else if (d == the_slave && can_synchronize()) 
  {
    //atomic operation
    __sync_fetch_and_or(&the_slave->sync_buf_clear_mask, 1 << ibuf); 
  }
  else //no synchronization needed, this is just the old code
  {

    USING(d); 
    int ret = 0; 
    uint8_t data_status[4]; 
    ret += buffer_append(d, buf_clear[1 << ibuf],0); 
    ret+=append_read_register(d, REG_CLEAR_STATUS, data_status); 
    ret+=buffer_send(d); //flush so we can clear the buffer immediately 
    if (data_status[3] & (1 << ibuf))
    {
      fprintf(stderr,"Did not clear buffer %d ? (or rate too high? buf mask after clearing: %x))\n", ibuf, data_status[3] & 0xf) ; 
      easy_break_point(); 
    }
    DONE(d); 
    return ret; 
  }

  uint8_t can_clear = the_master->sync_buf_clear_mask & the_slave->sync_buf_clear_mask; 

  while(can_clear) 
  {
    //get the lowest bit
    int buf2clr = __builtin_ctz(can_clear);

    uint8_t cleared_master[4]; 
    uint8_t cleared_slave[4]; 
    int ret = synchronized_command(buf_clear[1 << buf2clr], REG_CLEAR_STATUS, cleared_master, cleared_slave); 
//    printf("Clearing %d on both\n", buf2clr); 
    if (!ret)
    {
      if (cleared_master[3] & ( 1 << buf2clr))
      {
        fprintf(stderr,"Did not clear buffer %d for master ? (or rate too high? buf mask after clearing: %x))\n", buf2clr, cleared_master[3] & 0xf) ; 
        easy_break_point(); 
      }

      if (cleared_slave[3] & ( 1 << buf2clr))
      {
        fprintf(stderr,"Did not clear buffer %d for master ? (or rate too high? buf mask after clearing: %x))\n", buf2clr, cleared_slave[3] & 0xf) ; 
        easy_break_point(); 
      }

      if ((cleared_slave[3] & 0xf) != (cleared_master[3] & 0xf))
      {
        //TODO this might occasionally legimitately happen? maybe? 
        fprintf(stderr," master and slave free buffers don't match!: slave: 0x%x, master: 0x%x\n", cleared_slave[3] & 0xf, cleared_master[3] & 0xf); 
        easy_break_point(); 
      }
    //clear the bit
     can_clear&=~(1 << buf2clr); 
      __sync_fetch_and_and(&the_slave->sync_buf_clear_mask, ~(1 << buf2clr)); 
      __sync_fetch_and_and(&the_master->sync_buf_clear_mask, ~(1 << buf2clr)); 
    }
    else
    {
      fprintf(stderr,"Problem clearing stuff :(\n"); 
      return ret; 
    }
  }

  return 0;
}






static int loop_over_chunks_half_duplex(nuphase_dev_t * d, uint8_t naddr, uint8_t start_address, uint8_t * result) 
{

  int iaddr; 
  int ichunk; 
  int ret = 0; 

  for (iaddr = 0; iaddr < naddr; iaddr++) 
  {
    ret += buffer_append(d, buf_ram_addr[start_address + iaddr], 0); 
    if (ret) return ret; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      ret+= buffer_append(d, buf_chunk[ichunk], 0); 
      if (ret) return ret; 

      ret+= buffer_append(d, 0, result + NP_NUM_CHUNK *NP_SPI_BYTES* iaddr + ichunk * NP_SPI_BYTES); 
      if (ret) return ret; 
    }
  }

  return 0; 
}

static int __attribute__((unused)) 
loop_over_chunks_full_duplex(nuphase_dev_t * d, uint8_t naddr, uint8_t start_address, uint8_t * result)  
{

  int iaddr; 
  int ichunk; 
  int ret = 0; 

  for (iaddr  =0; iaddr < naddr; iaddr++) 
  {
    ret += buffer_append(d, buf_ram_addr[start_address + iaddr], 0); 
    if (ret) return ret; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      ret+= buffer_append(d, buf_chunk[ichunk], iaddr == 0 && ichunk == 0 ? 0 : result + NP_NUM_CHUNK * NP_SPI_BYTES * iaddr + (ichunk-1) * NP_SPI_BYTES); 
      if (ret) return ret; 

      if (iaddr == naddr-1 && ichunk == NP_NUM_CHUNK - 1)
      {
        ret+= buffer_append(d, 0, result + NP_NUM_CHUNK *NP_SPI_BYTES* iaddr + ichunk * NP_SPI_BYTES); 
        if (ret) return ret; 
      }
    }
  }

  return 0; 
}



int nuphase_read_raw(nuphase_dev_t *d, uint8_t buffer, uint8_t channel, uint8_t start, uint8_t finish, uint8_t * data) 
{

  uint8_t naddress = finish - start + 1; 
  int ret = 0; 

  USING(d);  //have to lock for the duration otherwise channel /read mode may be changed form underneath us.  
            // we don't lock before these because there is no way we sent enough transfers to trigger a read 
            //
  ret += buffer_append(d, buf_mode[MODE_WAVEFORMS], 0);  if (ret) return 0; 
  d->current_mode = MODE_WAVEFORMS; 
  ret += buffer_append(d, buf_buffer[buffer], 0);  if (ret) return 0; 
  d->current_buf = buffer; 
  ret += buffer_append(d, buf_channel[channel], 0);  if (ret) return 0; 
  ret += loop_over_chunks_half_duplex(d, naddress, start, data);
  if(!ret) ret = buffer_send(d); //pick up the stragglers. 
  DONE(d);  

  return ret; 
}



int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t *result)
{

  int ret; 
  if (address > NP_ADDRESS_MAX) return -1; 
  USING(d); 
  ret =  append_read_register(d, address,result); 
  ret += buffer_send(d); 
  DONE(d);
  return ret; 
}


int nuphase_sw_trigger(nuphase_dev_t * d ) 
{
  const uint8_t buf[4] = { REG_FORCE_TRIG, 0,0, 1};  
  int ret = 0; 

  if (d) 
  {
    USING(d); 
    ret = do_write(d->spi_fd, buf); 
    DONE(d); 
  }

  else if(can_synchronize())
  {
    ret = synchronized_command(buf,0,0,0); 
  }
  else
  {
    return -1; 
  }
  return ret == NP_SPI_BYTES ? 0 : -1;  
}

int nuphase_calpulse(nuphase_dev_t * d, unsigned state) 
{
  uint8_t buf[4] = { REG_CALPULSE, 0,0, state & 0xff };  
  int ret;
  USING(d); 
  ret = do_write(d->spi_fd, buf); 
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
    close(fd); 
    return 0; 
  }

  dev = malloc(sizeof(nuphase_dev_t)); 
  dev->device_name = devicename; 
  dev->spi_fd =fd;
  dev->spi_clock = SPI_CLOCK; 
  dev->cancel_wait = 0; 
  dev->event_counter = 0; 
  dev->next_read_buffer = 0; 
  dev->cs_change =NP_CS_CHANGE; 
  dev->delay_us =NP_DELAY_USECS; 
  dev->current_buf = -1; 
  dev->current_mode = -1; 

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
  uint8_t mode = SPI_MODE_0;  //we could change the chip select here too 
  ioctl(dev->spi_fd, SPI_IOC_WR_MODE, &mode); 
  ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &dev->spi_clock); 


  //configuration 
  if (c) dev->cfg = *c; 
  else nuphase_config_init(&dev->cfg); 

  //make sure sync is off 
  do_write(dev->spi_fd, buf_sync_off); 

  // if this is still running in 20 years, someone will have to fix the y2k38 problem 
  dev->readout_number_offset = ((uint64_t)time(0)) << 32; 
  dev->buffer_length = 624; 
  dev->channel_read_mask = 0xff; 
  dev->board_id = board_id_counter++; 

  dev->enable_locking = locking; 
  dev->nused = 0; 
  setup_xfers(dev); 

  if (locking) 
  {
    pthread_mutex_init(&dev->mut,0); 
    pthread_mutex_init(&dev->wait_mut,0); 

    //check if this is a master or slave if locking is enabled
    uint8_t fwver[4]; 
    nuphase_read_register(dev, REG_FIRMWARE_VER, fwver); 
    if (fwver[1])
    {
      if (the_master)
      {
        fprintf(stderr,"Warning: This is a master device, but a master device is already open!!! Ignore if not trying to sync two boards.\n"); 
      }
      else
      {
        the_master = dev; 
      }


      //set some things related to being master? 
    }
    else
    {
      if (the_slave) 
      {
        fprintf(stderr,"Warning: This is a slave device, but a slave device is already open!!! Ignore if not trying to sync two boards.\n"); 
      }
      else
      {
        the_slave = dev; 
      }
      //set some things related to being slave? 
    }
  }


  if (nuphase_reset(dev, &dev->cfg,NP_RESET_COUNTERS)) 
  {
    fprintf(stderr,"Unable to reset device... "); 
    nuphase_close(dev); 
    return 0; 
  }

  return dev; 

}

void nuphase_set_board_id(nuphase_dev_t * d, uint8_t id)
{
  if (id <= board_id_counter) board_id_counter = id+1; 
  d->board_id = id; 
}

uint8_t nuphase_get_board_id(const nuphase_dev_t * d) 
{
  return d->board_id; 
}

void nuphase_set_readout_number_offset(nuphase_dev_t * d, uint64_t offset) 
{
  d->readout_number_offset = offset; 
}


void nuphase_set_buffer_length(nuphase_dev_t * d, uint16_t length)
{
  d->buffer_length = length; 
}

uint16_t nuphase_get_bufferlength(const nuphase_dev_t * d) 
{
  return d->buffer_length; 
}


int nuphase_fwinfo(nuphase_dev_t * d, nuphase_fwinfo_t * info)
{

  //we need to read 5 registers, so 15 xfers 
  int ret = 0; 
  uint8_t version[NP_SPI_BYTES];
  uint8_t date[NP_SPI_BYTES];
  uint8_t dna_low[NP_SPI_BYTES]; 
  uint8_t dna_mid[NP_SPI_BYTES]; 
  uint8_t dna_hi[NP_SPI_BYTES]; 

  USING(d); 
  ret+=append_read_register(d, REG_FIRMWARE_VER, version); 
  ret+=append_read_register(d, REG_FIRMWARE_DATE, date); 
  ret+=append_read_register(d, REG_CHIPID_LOW, dna_low); 
  ret+=append_read_register(d, REG_CHIPID_MID, dna_mid); 
  ret+=append_read_register(d, REG_CHIPID_HI, dna_hi); 

  ret+=buffer_send(d); 
  DONE(d); 
  info->ver.major = version[3] >>4 ; 
  info->ver.minor = version[3] & 0x0f; 
  info->ver.master = version[1] & 1; 
  info->date.day = date[3] & 0xff; 
  info->date.month = date[2] & 0xf; 
  info->date.year = (date[2] >> 4) + (date[1] << 4); 

  //TODO check this logic. not sure endianness is correct
  uint64_t dna_low_big =  ((uint64_t) dna_low[3]) | ((uint64_t) dna_low[2]) << 8 || ((uint64_t) dna_low[1]) << 16;
  uint64_t dna_mid_big =  ((uint64_t) dna_mid[3]) | ((uint64_t) dna_mid[2]) << 8 || ((uint64_t) dna_mid[1]) << 16;
  uint64_t dna_hi_big =   ((uint64_t) dna_hi[3])  | ((uint64_t)dna_hi[2]) << 8 ;
  info->dna =  (dna_low_big & 0xffffff) | ( (dna_mid_big & 0xffffff) << 24) | ( (dna_hi_big & 0xffff) << 48); 

  return ret; 
}


int nuphase_close(nuphase_dev_t * d) 
{
  int ret = 0; 
  nuphase_cancel_wait(d); 
  USING(d); 

  //clear the buffer! 
  if (d->nused) 
  {
    ret+= buffer_send(d); 
  }

  if (d->enable_locking)
  {
    //this should be allowed? 
    pthread_mutex_unlock(&d->mut); 
    ret += 64* pthread_mutex_destroy(&d->mut); 

    if (pthread_mutex_trylock(&d->wait_mut)) // lock is beind held by a thread
    {
      nuphase_cancel_wait(d); 
      pthread_mutex_lock(&d->wait_mut); 
    }

    pthread_mutex_unlock(&d->wait_mut); 
    ret += 128* pthread_mutex_destroy(&d->wait_mut); 

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

void nuphase_cancel_wait(nuphase_dev_t *d) 
{
  d->cancel_wait = 1;  //this is necessary for polling and will sometimes catch the interrupt as well
  if (d->waiting_thread) 
  {
    syscall(SYS_tgkill, getpid(), d->waiting_thread, SIGINT);  //we need to send a SIGINT in case we are stuck inside poll. 
  }
}

// this is the most annoying (and probably buggiest) part of the code.
// it's difficult because we want to be  able to quickly cancel a 
//
int nuphase_wait(nuphase_dev_t * d, nuphase_buffer_mask_t * ready_buffers, float timeout) 
{

  //If locking is enabled and a second thread attempts
  //to wait for the same device, return EBUSY. 
  // making nuphase_wait for multiple threads sounds way too hard
  if (d->enable_locking) 
  {
    if (pthread_mutex_trylock(&d->wait_mut))
    {
      return EBUSY; 
    }
  }

  /* This was cancelled before (or almost concurrently with when)  we started.
  /  We'll just clear the cancel and return EAGAIN. I thought long and hard
  /  about what to do in the case that nuphase_cancel_wait is called and there
  /  is nothing waiting, but I think attempting to detect if nuphase_wait is
  /  currently called is fraught with race conditions. Anyway,
  /  nuphase_cancel_wait will probably not be called willy-nilly... usually
  /  just when you want to exit the program I imagine. 
  */
  
  if (d->cancel_wait) 
  {
    d->cancel_wait = 0; 
    return EAGAIN; 
  }



  //No gpio, we will just keep polling buffer mask
  // This is the simpler (but less efficient) case  
  if (!d->gpio_fd) 
  {
    nuphase_buffer_mask_t something = 0; 
    float waited = 0; 

    // keep trying until we either get something, are cancelled, or exceed our timeout (if we have a timeout) 
    while(!something && (timeout <= 0 || waited < timeout))
    {
      if (d->cancel_wait) break; 
      usleep(POLL_USLEEP); 
      waited += POLL_USLEEP * 1e-6; //us to s 
      something = nuphase_check_buffers(d,&d->hardware_next); 
    }
    int interrupted = d->cancel_wait; //were we interrupted? 

    if (ready_buffers) *ready_buffers = something;  //save to ready
    d->cancel_wait = 0;  //clear the wait
    if (d->enable_locking) pthread_mutex_unlock(&d->wait_mut);   //unlock the mutex
    return interrupted ? EINTR : 0; 
  }

  /*
  * If we are using the gpio interrupt, this becomes more complicated.  We
  * would block on the interrupt until we exceed our timeout (which may be
  * eternity). A signal would interrupt the call, but we may be called in a
  * context where the signal may not be available , or we might be in a thread
  * where the signal will not get deliver. 
  *
  * Hopefully what I'm doing makes sense. 
  */ 

  int ret = 0;


  // tell t he device which thread is waiting
  d->waiting_thread = syscall(SYS_gettid); 

  //we want to block all signals until 
  //we get to poll. Fewer things to worry about .. 
  sigset_t old;  //save the old signal mask to restore at the end
  sigset_t all; 
  sigfillset(&all); 
  pthread_sigmask(SIG_BLOCK, &all,&old); 

  //before we poll, check again to make sure we weren't already cancelled 
  if (d->cancel_wait)
  {
    ret = EINTR;
    goto cleanup; 
  }



  // tell ppoll to listen for SIGINT. Maybe this isn't the best signal, we can change it later. 
  sigset_t pollsigs; 
  sigfillset(&pollsigs); 
  sigdelset(&pollsigs,SIGINT); 

  //set up what we need for ppoll
  struct timespec ts; 
  ts.tv_sec = (int) timeout; 
  ts.tv_nsec = (timeout - ts.tv_sec) * 1e9; 
  struct pollfd fds = { .fd = d->gpio_fd, .events = POLLIN }; 

  //call ppoll. It will tell us if we got an interrupt, or timeout, or get interrupted (hopefully) by SIGINT 
  ret = ppoll(&fds,1, timeout <=0 ? 0 : &ts ,&pollsigs);  

  if (ret == 0)  // we timed out. 
  {
    if (*ready_buffers) *ready_buffers = 0;
    goto cleanup; 
  }

  if (ret < 0)
  {
    //this means we got interrupted, I think. 
    ret = errno; 
    goto cleanup; 
  }


  //alright, now we have to read the interrupt
  uint32_t info; 
  int nb = read(d->gpio_fd,&info, sizeof(info)); 

  //if we read it successfully, we'll unmask it.
  if (nb  == sizeof(info)) 
  {
    //let's unmask the interrupt
    uint32_t unmask = 1;
    nb = write(d->gpio_fd,&unmask,sizeof(unmask));
    if (nb < 0) 
    {
      ret = errno;
      fprintf(stderr,"Couldn't unmask interrupt, and I'm going to leave it in a bad state :( \n"); 
      goto cleanup; 
    }
  }
  else
  {
    ret = errno; 
    fprintf(stderr,"Couldn't read from interrupt, and I'm going to leave it in a bad sate :( \n"); 
    goto cleanup; 
  }

 //if we made it this far, this means we unmasked the interrupt, we probably got something, 
 if (ready_buffers) *ready_buffers = nuphase_check_buffers(d,&d->hardware_next); 
 ret = 0;  //we want to return success

 cleanup: 
 //we had an oops, so we haven't filled this yet. fill it with 0.
 if (ret && ready_buffers) *ready_buffers = 0; 

 //restore signal mask 
 pthread_sigmask(SIG_SETMASK, &old,0); 

 //we aren't waiting any more
 d->waiting_thread = 0; 
 //clear the wait
 d->cancel_wait = 0; 

 //unlock mutex
 if (d->enable_locking) pthread_mutex_unlock(&d->wait_mut); 

 return ret;

}



nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t * d, uint8_t * next) 
{

  uint8_t result[NP_SPI_BYTES]; 
  uint8_t result2[NP_SPI_BYTES]; 
  nuphase_buffer_mask_t mask; 
  int ret = 0; 

  USING(d); 
  //check twice ... 
  ret+=append_read_register(d, REG_STATUS, result); 
  ret+=append_read_register(d, REG_STATUS, result2); 
  ret+= buffer_send(d); 
  DONE(d); 
  mask  = result[3] &  BUF_MASK; // only keep lower 4 bits.
  if (mask != (result2[3] & BUF_MASK))
  {
    fprintf(stderr,"Got different answers mask0: %x mask1: %x next0: %d, next1: %d \n", mask, result2[3] & BUF_MASK, (result[2] >> 4) & 0x3,  (result2[2] >> 4) & 0x3); 

  }
  if (next) *next = (result[2] >> 4) & 0x3; 
  return mask; 
}

/* This just sets defaults */ 
void nuphase_config_init(nuphase_config_t * c) 
{
  int i;
  c->channel_mask = 0xff; 
  c->pretrigger = 1; //?? 
  c->trigger_mask = 0x7fff; 

  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    c->trigger_thresholds[i] = 0xfffff;  //TODO make this more sensible by default 
  }

  for (i = 0; i < NP_NUM_CHAN; i++)
  {
    c->attenuation[i] = 0;  //TODO make this more sensible by default 
  }
}


int nuphase_configure(nuphase_dev_t * d, const nuphase_config_t *c, int force ) 
{
  int written; 
  int ret = 0; 
  
  USING(d); 
  if (force || c->pretrigger != d->cfg.pretrigger)
  {
    uint8_t pretrigger_buf[] = { REG_PRETRIGGER, 0, 0, c->pretrigger}; 
    written = do_write(d->spi_fd, pretrigger_buf); 
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

  if (force || c->channel_mask!= d->cfg.channel_mask)
  {
    uint8_t channel_mask_buf[]= { REG_CHANNEL_MASK, 0, 0, c->channel_mask}; 
    written = do_write(d->spi_fd, channel_mask_buf); 
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

  if (force || c->trigger_mask != d->cfg.trigger_mask)
  {
    //TODO check the byte order
    uint8_t trigger_mask_buf[]= { REG_TRIGGER_MASK, 0, (c->trigger_mask >> 8) & 0xff, c->trigger_mask & 0xff}; 
    written = do_write(d->spi_fd, trigger_mask_buf); 
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

  if (force || memcmp(c->trigger_thresholds, d->cfg.trigger_thresholds, sizeof(c->trigger_thresholds)))
  {
    uint8_t thresholds_buf[NP_NUM_BEAMS][NP_SPI_BYTES]; 
    int i; 
    for (i = 0; i < NP_NUM_BEAMS; i++)
    {
      thresholds_buf[i][0]= REG_THRESHOLDS+i ;
      thresholds_buf[i][1]= (c->trigger_thresholds[i] >> 16 ) & 0xf;
      thresholds_buf[i][2]= ( c->trigger_thresholds[i] >> 8) & 0xff; 
      thresholds_buf[i][3]= c->trigger_thresholds[i] & 0xff;
      ret += buffer_append (d,thresholds_buf[i],0); 
    }
    
    ret += buffer_send(d); 

    if(!ret)
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

  if (force || memcmp(c->attenuation, d->cfg.attenuation, sizeof(c->attenuation)))
  {

    uint8_t attenuation_012[NP_SPI_BYTES] = { REG_ATTEN_012, c->attenuation[2], c->attenuation[1], c->attenuation[0] }; //TODO check order 
    uint8_t attenuation_345[NP_SPI_BYTES] = { REG_ATTEN_345, c->attenuation[5], c->attenuation[4], c->attenuation[3] }; //TODO check order 
    uint8_t attenuation_067[NP_SPI_BYTES] = { REG_ATTEN_67, 0x0, c->attenuation[7], c->attenuation[6] }; //TODO check order 


    ret += buffer_append(d,attenuation_012,0); 
    ret += buffer_append(d, attenuation_345,0); 
    ret += buffer_append(d, attenuation_067,0); 
    ret += buffer_append(d, buf_apply_attenuator,0); 

    ret += buffer_send(d); 
    if (!ret)
    {
      //success! 
      memcpy(d->cfg.attenuation, c->attenuation, sizeof(c->attenuation)); 
    }
    else
    {
      ret = -1; 
      goto configure_end; 
    }

  }

  if (force || c->trigger != d->cfg.trigger)
  {
    uint8_t trigger_enable_buf[NP_SPI_BYTES] = {REG_TRIG_ENABLE, 0, 0, c->trigger}; 

    written = do_write(d->spi_fd, trigger_enable_buf); 
    if (written == NP_SPI_BYTES) 
    {
      d->cfg.trigger = c->trigger; 
    }
    else
    {
      ret = -1; //don't try to continue if we fail . 
      goto configure_end; 
    }
  }

  if (force || c->trigger_holdoff != d->cfg.trigger_holdoff)
  {
    uint8_t trigger_holdoff_buf[NP_SPI_BYTES] = {REG_TRIG_HOLDOFF, 0, 0, c->trigger_holdoff}; 

    written = do_write(d->spi_fd, trigger_holdoff_buf); 
    if (written == NP_SPI_BYTES) 
    {
      d->cfg.trigger_holdoff = c->trigger_holdoff; 
    }
    else
    {
      ret = -1; //don't try to continue if we fail . 
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
  if (!nuphase_wait(d,&mask,-1) && mask) 
  {
    int ret; 
//    printf("dev->next_read_buffer: %d, mask after waiting: %x, hw_next: %d\n",d->next_read_buffer, mask, d->hardware_next); 
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



//lazy error checking macro 
#define CHK(X) if (X) { ret++; goto the_end; } 




int nuphase_read_multiple_ptr(nuphase_dev_t * d, nuphase_buffer_mask_t mask, nuphase_header_t ** hd, nuphase_event_t ** ev)
{
  int ibuf,ichan,ibeam;
  int iout = 0; 
  int ret = 0; 
  struct timespec now; 

  // we need to store some stuff in an intermediate format 
  // prior to putting into the header since the bits don't match 
  uint32_t event_counter[2]; 
  uint32_t trig_counter[2]; 
  uint32_t trig_time[2]; 
  uint32_t deadtime; 
  uint32_t tmask; 
  uint32_t tinfo; 

  int iibuf; 


  for (iibuf = 0; iibuf < __builtin_popcount(mask); iibuf++)
  {

    ibuf = d->next_read_buffer; 
    //we are not reading this event right now
    if ( (mask & (1 << ibuf)) == 0)
    {
      fprintf(stderr,"Sync issue? d->next_read_buffer=%d, mask=0x%x, hardware next: %d\n", d->next_read_buffer, mask, d->hardware_next); 
      easy_break_point(); 
      d->next_read_buffer =  __builtin_ctz(mask); //pick the lowest buffer to read next 
      break; 
    }

    clock_gettime(CLOCK_REALTIME, &now); 

    //grab the metadata 
    //set the buffer 
    USING(d); 
    d->event_counter++; 
    d->next_read_buffer = (d->next_read_buffer + 1) %NP_NUM_BUFFER; 
    CHK(buffer_append(d, buf_buffer[ibuf],0)) 
    d->current_buf = ibuf; 

    /**Grab metadata! */ 
    //switch to register mode  

    //we will pretend like we are bigendian so we can just call be64toh on the u64
    CHK(append_read_register(d,REG_EVENT_COUNTER_LOW, (uint8_t*) &event_counter[0])) 
    CHK(append_read_register(d,REG_EVENT_COUNTER_HIGH, (uint8_t*) &event_counter[1])) 
    CHK(append_read_register(d,REG_TRIG_COUNTER_LOW, (uint8_t*) &trig_counter[0])) 
    CHK(append_read_register(d,REG_TRIG_COUNTER_HIGH,(uint8_t*)  &trig_counter[1])) 
    CHK(append_read_register(d,REG_TRIG_TIME_LOW,(uint8_t*)  &trig_time[0])) 
    CHK(append_read_register(d,REG_TRIG_TIME_HIGH,(uint8_t*)  &trig_time[1])) 
    CHK(append_read_register(d,REG_DEADTIME, (uint8_t*) &deadtime)) 
    CHK(append_read_register(d,REG_TRIG_INFO, (uint8_t*) &tinfo)) 
    CHK(append_read_register(d,REG_TRIG_MASKS,(uint8_t*) &tmask)) 

    for(ibeam = 0; ibeam < NP_NUM_BEAMS; ibeam++)
    {
      CHK(append_read_register(d, REG_BEAM_POWER+ibeam, (uint8_t*)  &hd[iout]->beam_power[ibeam]))
    }
    //flush the metadata .  we could get slightly faster throughput by storing metadata 
    //read locations for each buffer and not flushing.
    // If it ends up mattering, I'll change it. 
    CHK(buffer_send(d)); 
    DONE(d);//yield  

#ifdef DEBUG_PRINTOUTS
    printf("Raw tinfo: %x\n", tinfo) ;
    printf("Raw tmask: %x\n", tmask) ;
    printf("Raw event_counter: %x %x\n", event_counter[0], event_counter[1]) ;
    printf("Raw trig_counter: %x %x\n", trig_counter[0], trig_counter[1]) ;
    printf("Raw trig_time: %x %x \n", trig_time[0], trig_time[1]) ;
#endif 
    
    // check the event counter
    event_counter[0] = be32toh(event_counter[0]) & 0xffffff; 
    event_counter[1] = be32toh(event_counter[1]) & 0xffffff; 
    trig_counter[0] = be32toh(trig_counter[0]) & 0xffffff; 
    trig_counter[1] = be32toh(trig_counter[1]) & 0xffffff; 
    trig_time[0] = be32toh(trig_time[0]) & 0xffffff; 
    trig_time[1] = be32toh(trig_time[1]) & 0xffffff; 

    uint64_t big_event_counter = event_counter[0] + (event_counter[1] << 24); 

    if (d->event_counter !=  big_event_counter)
    {
      fprintf(stderr,"Event counter mismatch!!! (sw: %"PRIu64", hw: %"PRIu64")\n", d->event_counter, big_event_counter); 
      easy_break_point(); 
    }

    //now fill in header data 
    tinfo = be32toh(tinfo); 
    tmask = be32toh(tmask); 

    uint8_t hwbuf =  (tinfo >> 22) & 0x3; 
    if ( hwbuf  != ibuf)
    {
      fprintf(stderr,"Buffer number mismatch!!! (sw: %u, hw: %u)\n", ibuf, hwbuf ); 
      easy_break_point(); 
    }
    
    hd[iout]->event_number = 0; //will be assigned later
    ev[iout]->event_number = 0; //will be assigned later
    hd[iout]->readout_number = d->readout_number_offset + big_event_counter; 
    ev[iout]->readout_number = hd[iout]->readout_number; 
    hd[iout]->trig_number = trig_counter[0] + (trig_counter[1] << 24); 
    hd[iout]->buffer_length = d->buffer_length; 
    hd[iout]->pretrigger_samples = d->cfg.pretrigger* 8 * 16; //TODO define these constants somewhere
    hd[iout]->readout_time = now.tv_sec; 
    hd[iout]->readout_time_ns = now.tv_nsec; 
    hd[iout]->trig_time =trig_time[0] + (trig_time[1] << 24); 

    hd[iout]->approx_trigger_time= d->start_time.tv_sec + hd[iout]->trig_time / BOARD_CLOCK_HZ; 
    hd[iout]->approx_trigger_time_nsecs = d->start_time.tv_nsec + (hd[iout]->trig_time % BOARD_CLOCK_HZ) *(1.e9 / BOARD_CLOCK_HZ); 
    if (hd[iout]->approx_trigger_time_nsecs > 1e9) 
    {
      hd[iout]->approx_trigger_time++; 
      hd[iout]->approx_trigger_time_nsecs-=1e9; 
    }

    hd[iout]->triggered_beams = tinfo & 0x7fff; 
    hd[iout]->beam_mask = tmask & 0x7fff;  
    for (ibeam = 0; ibeam < NP_NUM_BEAMS; ibeam++)
    {
      hd[iout]->beam_power[ibeam] = be32toh(hd[iout]->beam_power[ibeam]) & 0xffffff; 

    }
    hd[iout]->deadtime = be32toh(deadtime) & 0xffffff; 
    hd[iout]->buffer_number = hwbuf; 
    hd[iout]->channel_mask = (tmask >> 15) & 0xff; 
    hd[iout]->channel_overflow = 0; //TODO not implemented yet 
    hd[iout]->buffer_mask = mask; //this is the current buffer mask
    hd[iout]->board_id = d->board_id; 
    hd[iout]->trig_type = (tinfo >> 15) & 0x3; 
    hd[iout]->calpulser = (tinfo >> 21) & 0x1; 



    //and some event data 
    ev[iout]->buffer_length = d->buffer_length; 
    ev[iout]->board_id = d->board_id; 


    //now start to read the data 
    for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
    {
      USING(d);  
      if (d->current_mode != MODE_WAVEFORMS)
      {
        CHK(buffer_append(d, buf_mode[MODE_WAVEFORMS],0))
        d->current_mode = MODE_WAVEFORMS; 
      }

      if (d->current_buf != ibuf)
      {
        CHK(buffer_append(d, buf_mode[MODE_WAVEFORMS],0))
      }

      if (d->channel_read_mask & (1 << ichan)) //TODO is this backwards?!??? 
      {
        CHK(buffer_append(d, buf_channel[ichan],0)) 
        CHK(loop_over_chunks_half_duplex(d, d->buffer_length / (NP_SPI_BYTES * NP_NUM_CHUNK),1, ev[iout]->data[ichan]))
      }
      else
      {
        memset(ev[iout]->data[ichan], 0 , hd[iout]->buffer_length); 
      }
      DONE(d); 
    }

    mark_buffer_done(d, ibuf); 

    iout++; 
  }


  the_end:
  //TODO add some printout here in case of falure/ 
  //also, need to unlock mutex if it's locked 

  return ret; 
}


int nuphase_clear_buffer(nuphase_dev_t *d, nuphase_buffer_mask_t mask) 
{
  int ret; 
  USING(d); 
  ret = do_write(d->spi_fd, buf_clear[mask & BUF_MASK]); //TODO check if this is really the right interface 
  DONE(d); 
  return ret == NP_SPI_BYTES ? 0 : -1; 
}

int nuphase_write(nuphase_dev_t *d, const uint8_t* buffer)
{
  int written = 0; 
  USING(d); 
  written = do_write(d->spi_fd, buffer); 
  DONE(d); 
  return written == NP_SPI_BYTES ? 0 : -1; 
}

int nuphase_read(nuphase_dev_t *d,uint8_t* buffer)
{
  int got = 0; 
  USING(d); 
  got = do_read(d->spi_fd, buffer); 
  DONE(d); 
  return got == NP_SPI_BYTES ? 0 : -1; 
}



int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * st) 
{
  //TODO: fill in deadtime when I figure out how. 
  int i; 
  int ret = 0; 
  struct timespec now; 
  uint8_t wide_scalers[NP_NUM_BEAMS][NP_SPI_BYTES]; 

  st->board_id = d->board_id; 

  USING(d); 
  ret+=buffer_append(d, buf_mode[MODE_REGISTER],0); 
  d->current_mode = MODE_REGISTER; 
  ret+=buffer_append(d, buf_update_scalers,0); 

  for (i = 0; i < NP_NUM_BEAMS; i++) 
  {
    ret+=buffer_append(d, buf_pick_scaler[i],0); 
    ret+=append_read_register(d, REG_SCALER_READ, wide_scalers[i]); 
  }

  clock_gettime(CLOCK_REALTIME, &now); 
  ret+= buffer_send(d); 
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

//todo there is probably a simpler way to calculate this... 
static struct timespec avg_time(struct timespec A, struct timespec B)
{
  struct timespec avg; 
  avg.tv_nsec = (A.tv_nsec + B.tv_nsec) /2; 
  uint32_t tmp_sum = A.tv_sec + B.tv_sec; 
  avg.tv_sec = tmp_sum/2; 
  if (tmp_sum % 2 == 1 ) 
  {
    avg.tv_nsec += 5e8; 
  }

  if (avg.tv_nsec > 1e9) 
  {
    avg.tv_sec++; 
    avg.tv_nsec-=1e9; 
  }

  return avg; 
}




/*  this has slowly grown into a bit of a monstrosity, since it is responsible 
 *  for all the different reset modes. 
 *
 *
 */
int nuphase_reset(nuphase_dev_t * d,const  nuphase_config_t * c, nuphase_reset_t reset_type)
{
  
  int wrote; 

  // We start by tickling the right reset register
  // if we are doing a global, almost global or ADC reset. 
  // We need to verify that these sleep delays are good.
  
  if (reset_type == NP_RESET_GLOBAL) 
  {
    wrote = do_write(d->spi_fd, buf_reset_all); 
    if (wrote != NP_SPI_BYTES) 
    {
      return 1;
    }
    fprintf(stderr,"Full reset...\n"); 
    //we need to sleep for a while. how about 20 seconds? 
    sleep(20); 
    fprintf(stderr,"...done\n"); 
  }
  else if (reset_type == NP_RESET_ALMOST_GLOBAL)
  {
    wrote = do_write(d->spi_fd, buf_reset_almost_all); 
    if (wrote != NP_SPI_BYTES) 
    {
      return 1;
    }
    fprintf(stderr,"Almost full reset...\n"); 
    //we need to sleep for a while. how about 20 seconds? 
    sleep(20); 
    fprintf(stderr,"...done\n"); 
  }

  if (reset_type == NP_RESET_ADC)
  {
    wrote = do_write(d->spi_fd, buf_reset_adc); 
    if (wrote != NP_SPI_BYTES) 
    {
      return 1;
    }
    sleep(10); // ? 
  }

  /* afer all resets (if applicable), we want to restart the event counter 
   * and, if any of the stronger resets were applied, apply the calibration. 
   *
   * The order of operations is: 
   *
   *  - turn off all trigger masks
   *  - clear allthe buffers 
   *  - if necessary, do the calibration 
   *  - reset the event / trig time counters (and save the time to try to match it up later) 
   *  - call nuphase_configure with the passed config. 
   *  
   
   **/

  //start by clearing the masks 
  wrote = do_write( d->spi_fd, buf_clear_all_masks);

  if (wrote != NP_SPI_BYTES) 
  {
      fprintf(stderr, "Unable to clear masks. Aborting reset\n"); 
      return 1; 
  }

  //clear all buffers
  wrote = write  (d->spi_fd, buf_clear[0xf], NP_SPI_BYTES); 

  if (wrote != NP_SPI_BYTES) 
  {
      fprintf(stderr, "Unable to clear buffers. Aborting reset\n"); 
      return 1; 
  }

  d->next_read_buffer = 0; //reset the next read buffer 

  //do the calibration, if necessary 
  
  /* The calibration proceeds as follows:
   *   - temporarily set the channel length to something long 
   *   - enable the calpulser 
   *   - until we are happy:  
   *      - send software trigger
   *      - read event
   *      - find peak value of each channel
   *      - make sure peak values are at least some size and not farther than 16
   *      - if all good, set delays accordingly 
   *
   *   disable the cal pulser
   */
  if (0 && reset_type >= NP_RESET_ADC)//temporar disable 
  {
    int happy = 0; 
    int misery = 0; 
    wrote = NP_SPI_BYTES; 

    //temporarily set the buffer length to the maximum 
    uint16_t old_buf_length = d->buffer_length; 
    d->buffer_length = NP_MAX_WAVEFORM_LENGTH; 

    //release the calpulser 
    nuphase_calpulse(d, 1); 

    while (!happy) 
    {
      if (misery++ > 0) 
      {
        wrote = do_write(d->spi_fd, buf_adc_clk_rst); 
        fprintf(stderr,"When adc_clk_rst, expected %d got %d\n", NP_SPI_BYTES, wrote);  
      }

      if (misery> 3) 
      {
        fprintf(stderr,"Misery now at %d\n", misery); 
      }

      if (misery > MAX_MISERY) 
      {
        fprintf(stderr,"Maximum misery reached. We can't take it anymore. Giving up on ADC alignment and not bothering to configure.\n"); 
        break; 
      }

      if (wrote < NP_SPI_BYTES) continue; //try again if the write failed 


      nuphase_buffer_mask_t mask; 
      nuphase_sw_trigger(d); 
      nuphase_wait(d,&mask,1); 
      int nbuf = __builtin_popcount(mask); 

      if (!nbuf)
      {
        fprintf(stderr,"no buffers ready after SW trigger... something's fishy. Trying again!\n"); 
        continue; 
      }

      if (nbuf > 1) 
      {
        fprintf(stderr,"that's odd, we should only have one buffer. Mask is : 0x%x\n", mask); 
      }


      //read in the first buffer (should really be  0 most of the time.) 
      nuphase_read_single(d, __builtin_ctz(mask),  &d->calib_hd, &d->calib_ev); 

      // now loop over the samples and get the things we need 
      uint16_t min_max_i = NP_MAX_WAVEFORM_LENGTH; 
      uint16_t max_max_i = 0; 
      uint8_t min_max_v = 255; 
      uint16_t max_i[NP_NUM_CHAN] = {0}; 

      //loop through and find where the maxes are
      int ichan, isamp; 

      for (ichan = 0; ichan <NP_NUM_CHAN; ichan++)
      {
        uint8_t max_v = 0; 
        for (isamp = 0; isamp < NP_MAX_WAVEFORM_LENGTH; isamp++)
        {
          if ( d->calib_ev.data[ichan][isamp] > max_v)
          {
            max_v = d->calib_ev.data[ichan][isamp]; 
            max_i[ichan] = isamp; 
          }
        }

        if (max_i[ichan] < min_max_i) min_max_i = max_i[ichan]; 
        if (max_i[ichan] > max_max_i)  max_max_i = max_i[ichan]; 
        if (max_v < min_max_v)  min_max_v = max_v; 
      }

      //sanity checks 

      if (min_max_v < MIN_GOOD_MAX_V) // TODO come up with a good value
      {
        fprintf(stderr,"Minimum Max V was %x. Did we get a pulse in each channel? \n",min_max_v) ;
        continue; 
      }

      //too much delay 
      if (max_max_i - min_max_i > 16) 
      {
        fprintf(stderr,"Maximum delay required is %d. Let's try again. \n",max_max_i - min_max_i) ;
        continue; 
      }

      int iadc; 
      //otherwise, we are in business! Take averages of channel for each adc
      for (iadc = 0; iadc < NP_NUM_CHAN/2; iadc++)
      {
        uint8_t delay  = (max_i[2*iadc] + max_i[2*iadc+1]- 2*min_max_i)/2; 
        uint8_t buf[NP_SPI_BYTES] = {REG_ADC_DELAYS + iadc, 0, 0, delay}; 
        wrote = do_write(d->spi_fd, buf); 
        if (wrote < NP_SPI_BYTES) 
        {
          fprintf(stderr,"Should have written %d but wrote %d\n", NP_SPI_BYTES, wrote); 
          continue;//why not? 
        }
      }

     //yay
      happy=1; 
    }

    d->buffer_length = old_buf_length; 
    nuphase_calpulse(d, 0); 
    if (!happy) return -1; 
  }

  //make sure we are starting on buffer zero
  nuphase_check_buffers(d, &d->hardware_next); 
  while (d->hardware_next)
  {
    nuphase_sw_trigger(d); 
    nuphase_check_buffers(d, &d->hardware_next); 
    nuphase_clear_buffer(d,0xf); 
  }
  
//  printf("Starting on buffer: %d\n", d->next_read_buffer); 


  //then reset the counters, measuring the time before and after 
   
   struct timespec tbefore; 
   struct timespec tafter; 
   clock_gettime(CLOCK_REALTIME,&tbefore); 
   wrote = do_write(d->spi_fd, buf_reset_counter); 
   clock_gettime(CLOCK_REALTIME,&tafter); 
    

   if (wrote != NP_SPI_BYTES) 
   {
      fprintf(stderr, "Unable to reset counters. Aborting reset\n"); 
      return 1; 
   }

    //take average for the start time
    d->start_time = avg_time(tbefore,tafter); 


   //finally we must configure it the way we like it 
   return nuphase_configure(d,c,1); 
}

int nuphase_set_spi_clock(nuphase_dev_t *d, unsigned clock) 
{
  d->spi_clock = clock*1000000; 
  USING(d); 
  ioctl(d->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &d->spi_clock); 
  DONE(d); 

  return 0; //check? 
}

int nuphase_set_toggle_chipselect(nuphase_dev_t *d, int cs) 
{
  d->cs_change = cs;
  setup_xfers(d); 
  return 0; 
}

int nuphase_set_transaction_delay(nuphase_dev_t *d, unsigned delay) 
{
  d->delay_us = delay;
  setup_xfers(d); 
  return 0; 
}

