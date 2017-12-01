
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
#include "bbb_gpio.h" 

#define NP_ADDRESS_MAX 128 
#define NP_SPI_BYTES  NP_WORD_SIZE
#define NP_NUM_MODE 4
#define NP_NUM_REGISTER 128
#define BUF_MASK 0xf
#define MAX_PRETRIGGER 8 
#define BOARD_CLOCK_HZ 25000000

// master + slave
#define NBD(d) (d->fd[1] ? 2 : 1)

#define MIN_GOOD_MAX_V 20 
#define MAX_MISERY 100 

#define SPI_CAST  (uintptr_t) 

#define NP_DELAY_USECS 0
#define NP_CS_CHANGE 0

#define SPI_CLOCK 20000000
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
  REG_LATCHED_PPS_LOW    = 0x2c, 
  REG_LATCHED_PPS_HIGH   = 0x2d, 
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
  REG_READ               = 0x47, //send data to spi miso 
  REG_EXT_INPUT_CONFIG   = 0x4b, 
  REG_PRETRIGGER         = 0x4c, 
  REG_CLEAR              = 0x4d, //clear buffers 
  REG_BUFFER             = 0x4e,
  REG_TRIGGER_MASK       = 0x50, 
  REG_TRIG_HOLDOFF       = 0x51, 
  REG_TRIG_ENABLE        = 0x52, 
  REG_TRIGOUT_CONFIG     = 0x53, 
  REG_PHASED_TRIGGER     = 0x54, 
  REG_VERIFICATION_MODE  = 0x55, 
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

struct nuphase_dev
{
  const char * device_name[2]; //msater,slave
  int fd[2];  //master, slave
  int power_gpio; //gpio for enable 
  int enable_locking; 
  uint64_t readout_number_offset; 
  uint64_t event_counter;  // should match device...we'll keep this to complain if it doesn't
  uint16_t buffer_length; 
  pthread_mutex_t mut; //mutex for the SPI (not for the gpio though). Only used if enable_locking is true
  pthread_mutex_t wait_mut; //mutex for the waiting. Only used if enable_locking is true
  uint8_t board_id[2]; 
  uint8_t channel_read_mask[2];// read mask... right now it's always 0xf, but we can make it configurable later
  volatile int cancel_wait; // needed for signal handlers 
  struct timespec start_time; //the time of the last clock reset

  uint8_t next_read_buffer; //what buffer to read next 
  uint8_t hardware_next; // what buffer the hardware things we should read next 

  uint32_t min_threshold; 
  uint16_t poll_interval; 
  int spi_clock; 
  int cs_change; 
  int delay_us; 

  uint8_t pretrigger; 

  // store event / header used for calibration here in case we want it later? 
  nuphase_event_t calib_ev;
  nuphase_header_t calib_hd;

  //spi buffer 
  struct spi_ioc_transfer buf[2][MAX_XFERS]; 

  int nused[2]; 

  // device state 
  int current_buf[2]; 
  int current_mode[2]; 

  bbb_gpio_pin_t * gpio_pin; 
  
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
  printf("START BULK TRANSFER (fd=%d, t = %lu.%lu)\n", fd,start.tv_sec, start.tv_nsec); 
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
  printf("END BULK TRANSFER (fd=%d,t= %lu.%lu)\n", fd,end.tv_sec, end.tv_nsec); 
#endif 
  return ret; 
}

static int do_write(int fd, const uint8_t * p)
{
  int ret = write(fd,p, NP_SPI_BYTES); 
#ifdef DEBUG_PRINTOUTS
  printf("WRITE(%d): [0x%02x 0x%02x 0x%02x 0x%02x]\n",fd, p[0],p[1],p[2],p[3]); 
#endif
  return ret; 
}

static int do_read(int fd, uint8_t * p)
{
  int ret = read(fd,p, NP_SPI_BYTES); 
#ifdef DEBUG_PRINTOUTS
  printf("READ(%d): [0x%02x 0x%02x 0x%02x 0x%02x]\n",fd, p[0],p[1],p[2],p[3]); 
#endif
  return ret; 
}



#define N_SCALER_REGISTERS  (NP_NUM_SCALERS * (1 + NP_NUM_BEAMS)/2) 

// all possible buffers we might batch
static uint8_t buf_mode[NP_NUM_MODE][NP_SPI_BYTES];
static uint8_t buf_set_read_reg[NP_NUM_REGISTER][NP_SPI_BYTES];
static uint8_t buf_channel[NP_NUM_CHAN][NP_SPI_BYTES];
static uint8_t buf_buffer[NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_chunk[NP_NUM_CHUNK][NP_SPI_BYTES];
static uint8_t buf_ram_addr[NP_ADDRESS_MAX][NP_SPI_BYTES];
static uint8_t buf_clear[1 << NP_NUM_BUFFER][NP_SPI_BYTES];
static uint8_t buf_reset_buf[NP_SPI_BYTES] = {REG_CLEAR,0,1,0};
static uint8_t buf_pick_scaler[N_SCALER_REGISTERS][NP_SPI_BYTES]; 

static uint8_t buf_read[NP_SPI_BYTES] __attribute__((unused))= {REG_READ,0,0,0}  ; 

static uint8_t buf_update_scalers[NP_SPI_BYTES] = {REG_UPDATE_SCALERS,0,0,1} ; 
static uint8_t buf_sync_on[NP_SPI_BYTES] = {REG_SYNC,0,0,1} ; 
static uint8_t buf_sync_off[NP_SPI_BYTES] = {REG_SYNC,0,0,0} ; 
static uint8_t buf_reset_all[NP_SPI_BYTES] = {REG_RESET_ALL,0,0,1}; 
static uint8_t buf_reset_almost_all[NP_SPI_BYTES] = {REG_RESET_ALL,0,0,2}; 
static uint8_t buf_reset_counter[NP_SPI_BYTES] = {REG_RESET_COUNTER,0,0,1}; 
static uint8_t buf_adc_clk_rst[NP_SPI_BYTES] = {REG_ADC_CLK_RST,0,0,0}; 
static uint8_t buf_apply_attenuation[NP_SPI_BYTES] = {REG_ATTEN_APPLY,0,0,0}; 

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
    buf_clear[i][0] = REG_CLEAR; 
    buf_clear[i][3] = i;  
  }

  memset(buf_pick_scaler,0,sizeof(buf_pick_scaler)); 
  for (i = 0; i < N_SCALER_REGISTERS; i++)
  {
    buf_pick_scaler[i][0]=REG_PICK_SCALER; 
    buf_pick_scaler[i][3]=i;  
  }
}



static void setup_xfers( nuphase_dev_t *d)
{
  int i, b; 
  USING(d); 
  for (b = 0; b < NBD(d); b++)
  {
    for (i = 0; i < MAX_XFERS; i++)
    {
      d->buf[b][i].len = NP_SPI_BYTES; 
      d->buf[b][i].cs_change =d->cs_change; //deactivate cs between transfers
      d->buf[b][i].delay_usecs = d->delay_us;//? 
    }
  }
  DONE(d); 
}


static int buffer_send(nuphase_dev_t * d, nuphase_which_board_t which)
{
  int wrote; 
  if (!d->nused[which]) return 0; 
  wrote = do_xfer(d->fd[which], d->nused[which], d->buf[which]); 
  if (wrote < d->nused[which] * NP_SPI_BYTES) 
  {
    fprintf(stderr,"IOCTL failed! returned: %d\n",wrote); 
    return -1; 
  }
  d->nused[which] = 0; 

  return 0; 
}



// this will send if full!  
static int buffer_append(nuphase_dev_t * d, nuphase_which_board_t which, const uint8_t * txbuf, const uint8_t * rxbuf) 
{
  //check if full 
  if (d->nused[which] >= MAX_XFERS) //greater than just in case, but it already means something went horribly wrong 
  {
    if (buffer_send(d,which))
    {
      return -1; 
    }
  }

  d->buf[which][d->nused[which]].tx_buf = SPI_CAST txbuf; 
  d->buf[which][d->nused[which]].rx_buf = SPI_CAST rxbuf; 
  d->nused[which]++; 
  return 0; 
}

static int append_read_register(nuphase_dev_t *d, nuphase_which_board_t which, uint8_t address, uint8_t * result)
{
  int ret = 0; 
  ret += buffer_append(d,which, buf_set_read_reg[address],0);
  ret += buffer_append(d,which,0,result); 
  return ret; 
}



/* internal synchronized command if reg_to_read_after is not zero, will read a
 * register after (for example to see if something worked) and store the result
 * in the appropriate place. 
 **/ 

static int synchronized_command(nuphase_dev_t *d, const uint8_t * cmd, uint8_t reg_to_read_after,
                                  uint8_t * result_master, uint8_t * result_slave) {
  
  //just do a normal command
  if (NBD(d) < 2) 
  {
    int ret =0; 
    USING(d); 
    ret += buffer_append(d,MASTER,cmd,0); 
    if (reg_to_read_after)
    {
      ret+=append_read_register(d,MASTER, reg_to_read_after, result_master); 
    }
    ret+= buffer_send(d,MASTER); 
    DONE(d); 
    return ret; 
  }

  USING(d); 

  int ret = 0;
  //send sync on to master
  ret+=buffer_append(d,MASTER, buf_sync_on,0); 
  ret+=buffer_send(d, MASTER); 
  //send command to slave 
  ret+=buffer_append(d, SLAVE, cmd,0); 
  ret+=buffer_send(d,SLAVE); 

  //send command, and then sync off to master
  ret+=buffer_append(d,MASTER, cmd,0); 
  ret+=buffer_append(d,MASTER, buf_sync_off,0); 
  ret+=buffer_send(d,MASTER); 


  if (reg_to_read_after) 
  {
    ret+=append_read_register(d, MASTER, reg_to_read_after, result_master); 
    ret+=append_read_register(d, SLAVE, reg_to_read_after, result_slave); 
    ret+=buffer_send(d,SLAVE); 
    ret+=buffer_send(d,MASTER); 
  }


  DONE(d); 
  return ret; 
}

static int mark_buffers_done(nuphase_dev_t * d,  nuphase_buffer_mask_t buf)
{

  if (NBD(d) < 2) //no slave device, so no sync needed
  {

    USING(d); 
    int ret = 0; 
    uint8_t data_status[4]; 
    ret += buffer_append(d, MASTER, buf_clear[buf],0); 
    ret+=append_read_register(d,MASTER,  REG_CLEAR_STATUS, data_status); 
    ret+=buffer_send(d,MASTER); //flush so we can clear the buffer immediately 
    if (data_status[3] & (buf))
    {
//      fprintf(stderr,"Did not clear buffer mask %x ? (or rate too high? buf mask after clearing: %x))\n", buf, data_status[3] & 0xf) ; 
 //     easy_break_point(); 
    }
    DONE(d); 
    return ret; 
  }


  else
  {
    uint8_t cleared_master[NP_SPI_BYTES]; 
    uint8_t cleared_slave[NP_SPI_BYTES]; 

    int ret = synchronized_command(d, buf_clear[buf], REG_CLEAR_STATUS, cleared_master, cleared_slave); 
//    printf("Clearing %d on both\n", buf2clr); 
    if (!ret)
    {
      if (cleared_master[3] & ( buf))
      {
//        fprintf(stderr,"Did not clear buffer mask %x for master ? (or rate too high? buf mask after clearing: %x))\n", buf, cleared_master[3] & 0xf) ; 
//       easy_break_point(); 
      }

      if (cleared_slave[3] & (buf))
      {
//        fprintf(stderr,"Did not clear buffer %x for master ? (or rate too high? buf mask after clearing: %x))\n", buf, cleared_slave[3] & 0xf) ; 
//       easy_break_point(); 
      }

//      if ((cleared_slave[3] & 0xf) != (cleared_master[3] & 0xf))
//      {
//        //TODO this might occasionally legimitately happen? maybe? 
//        fprintf(stderr," master and slave free buffers don't match!: slave: 0x%x, master: 0x%x\n", cleared_slave[3] & 0xf, cleared_master[3] & 0xf); 
//        easy_break_point(); 
//      }
    }
    else
    {
      fprintf(stderr,"Problem clearing stuff :(\n"); 
      return ret; 
    }
  }

  return 0;
}






static int loop_over_chunks_half_duplex(nuphase_dev_t * d, nuphase_which_board_t which,  uint8_t naddr, uint8_t start_address, uint8_t * result) 
{

  int iaddr; 
  int ichunk; 
  int ret = 0; 

  for (iaddr = 0; iaddr < naddr; iaddr++) 
  {
    ret += buffer_append(d,which, buf_ram_addr[start_address + iaddr], 0); 
    if (ret) return ret; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      ret+= buffer_append(d,which, buf_chunk[ichunk], 0); 
      if (ret) return ret; 

      ret+= buffer_append(d, which, 0, result + NP_NUM_CHUNK *NP_SPI_BYTES* iaddr + ichunk * NP_SPI_BYTES); 
      if (ret) return ret; 
    }
  }

  return 0; 
}

static int __attribute__((unused)) 
loop_over_chunks_full_duplex(nuphase_dev_t * d, nuphase_which_board_t which,  uint8_t naddr, uint8_t start_address, uint8_t * result)  
{

  int iaddr; 
  int ichunk; 
  int ret = 0; 

  for (iaddr  =0; iaddr < naddr; iaddr++) 
  {
    ret += buffer_append(d,which, buf_ram_addr[start_address + iaddr], 0); 
    if (ret) return ret; 

    for (ichunk = 0; ichunk < NP_NUM_CHUNK; ichunk++)
    {
      ret+= buffer_append(d,which, buf_chunk[ichunk], iaddr == 0 && ichunk == 0 ? 0 : result + NP_NUM_CHUNK * NP_SPI_BYTES * iaddr + (ichunk-1) * NP_SPI_BYTES); 
      if (ret) return ret; 

      if (iaddr == naddr-1 && ichunk == NP_NUM_CHUNK - 1)
      {
        ret+= buffer_append(d, which, 0, result + NP_NUM_CHUNK *NP_SPI_BYTES* iaddr + ichunk * NP_SPI_BYTES); 
        if (ret) return ret; 
      }
    }
  }

  return 0; 
}



int nuphase_read_raw(nuphase_dev_t *d,  uint8_t buffer, uint8_t channel, uint8_t start, uint8_t finish, uint8_t * data, nuphase_which_board_t which) 
{

  uint8_t naddress = finish - start + 1; 
  int ret = 0; 

  USING(d);  //have to lock for the duration otherwise channel /read mode may be changed form underneath us.  
            // we don't lock before these because there is no way we sent enough transfers to trigger a read 
            //
  ret += buffer_append(d,which, buf_mode[MODE_WAVEFORMS], 0);  if (ret) return 0; 
  d->current_mode[which] = MODE_WAVEFORMS; 
  ret += buffer_append(d,which, buf_buffer[buffer], 0);  if (ret) return 0; 
  d->current_buf[which] = buffer; 
  ret += buffer_append(d,which, buf_channel[channel], 0);  if (ret) return 0; 
  ret += loop_over_chunks_half_duplex(d,which, naddress, start, data);
  if(!ret) ret = buffer_send(d,which); //pick up the stragglers. 
  DONE(d);  

  return ret; 
}



int nuphase_read_register(nuphase_dev_t * d, uint8_t address, uint8_t *result, nuphase_which_board_t which)
{

  int ret; 
  if (address > NP_ADDRESS_MAX) return -1; 
  USING(d); 
  ret =  append_read_register(d,which, address,result); 
  ret += buffer_send(d,which); 
  DONE(d);

  if ( result[0] != address) 
  {
    fprintf(stderr,"WARNING: read register mismatch. Expected 0x%x, got 0x%x\n", address, result[0]); 
    ret++; 
  }

  return ret; 
}


int nuphase_sw_trigger(nuphase_dev_t * d ) 
{
  const uint8_t buf[4] = { REG_FORCE_TRIG, 0,0, 1};  
  int ret = 0; 


  if (NBD(d) < 2) 
  {
    USING(d); 
    int wrote; 
    wrote = do_write(d->fd[0], buf); //always the master
    ret = wrote == NP_SPI_BYTES ? 0 : -1;  
    DONE(d); 
  }
  else
  {
    ret = synchronized_command(d, buf, 0,0,0); 
  }


  return ret; 
}


int nuphase_calpulse(nuphase_dev_t * d, unsigned state) 
{
  uint8_t buf[4] = { REG_CALPULSE, 0,0, state};  
  int ret;
  USING(d); 
  ret = do_write(d->fd[0], buf); 
  ret = do_write(d->fd[1], buf); 
  DONE(d); 
  return ret == NP_SPI_BYTES ? 0 : 1 ;
}


static int board_id_counter =1; 

nuphase_dev_t * nuphase_open(const char * devicename_master,
                             const char * devicename_slave,
                             int gpio_number, int locking)
{
  int locked,fd[2]; 
  nuphase_dev_t * dev; 

  fd[0] = open(devicename_master, O_RDWR); 
  if (fd[0] < 0) 
  {
    fprintf(stderr,"Could not open %s\n", devicename_master); 
    return 0; 
  }

  locked = flock(fd[0],LOCK_EX | LOCK_NB); 
  if (locked < 0) 
  {
    fprintf(stderr,"Could not get exclusive access to %s\n", devicename_master); 
    close(fd[0]); 
    return 0; 
  }




  if (devicename_slave) 
  {
    fd[1] = open(devicename_slave, O_RDWR); 
    if(fd[1] < 0) 
    {

      fprintf(stderr, "Could not open %s\n" ,devicename_slave); 
      close(fd[0]); 
      return 0; 
    }

    locked = flock(fd[1], LOCK_EX | LOCK_NB); 
    if (locked < 0) 
    {

      fprintf(stderr,"Could not get exclusive access to %s\n", devicename_master); 
      close(fd[0]); 
      close(fd[1]); 
      return 0; 
    }
  }
  else
  {
    fd[1] = 0; 
  }


  bbb_gpio_pin_t * gpio_pin = 0;

  if (gpio_number) 
  {
    gpio_pin = bbb_gpio_open(gpio_number); 
    bbb_gpio_set(gpio_pin,0); 
  }

  //make sure sync is off 
  do_write(fd[0], buf_sync_off); 



  dev = malloc(sizeof(nuphase_dev_t)); 
  dev->poll_interval = 500; 
  memset(dev,0,sizeof(*dev)); 
  dev->gpio_pin = gpio_pin; 
  dev->device_name[0] = devicename_master; 
  dev->device_name[1] = devicename_slave; 
  memcpy(dev->fd,fd,sizeof(fd));
  dev->spi_clock = SPI_CLOCK; 
  dev->cancel_wait = 0; 
  dev->event_counter = 0; 
  dev->next_read_buffer = 0; 
  dev->cs_change =NP_CS_CHANGE; 
  dev->delay_us =NP_DELAY_USECS; 
  dev->current_buf[0] = -1; 
  dev->current_buf[1] = -1; 
  dev->current_mode[0] = -1; 
  dev->current_mode[1] = -1; 

  dev->min_threshold = 5000; 

  //Configure the SPI protocol 
  uint8_t mode = SPI_MODE_0;  //we could change the chip select here too 
  int ifd = 0;

  for (ifd = 0; ifd < NBD(dev); ifd++)
  {
      ioctl(dev->fd[ifd], SPI_IOC_WR_MODE, &mode); 
      ioctl(dev->fd[ifd], SPI_IOC_WR_MAX_SPEED_HZ, &dev->spi_clock); 
  }

  // if this is still running in 20 years, someone will have to fix the y2k38 problem 
  dev->readout_number_offset = ((uint64_t)time(0)) << 32; 
  dev->buffer_length = 624; 
  dev->channel_read_mask[0] = 0xff; 
  dev->channel_read_mask[1] = fd[1] ? 0xf : 0; 
  dev->board_id[0] = board_id_counter++; 
  dev->board_id[1] = fd[1] ? board_id_counter++ : 0; 

  dev->enable_locking = locking; 
  memset(dev->nused,0,sizeof(dev->nused)); 
  setup_xfers(dev); 

  if (locking) 
  {
    pthread_mutex_init(&dev->mut,0); 
    pthread_mutex_init(&dev->wait_mut,0); 
  }


 //check if this is a master or slave if locking is enabled
 uint8_t fwver[4]; 
 nuphase_read_register(dev, REG_FIRMWARE_VER, fwver, MASTER); 

 if (!fwver[1])
 {
   fprintf(stderr,"WARNING! The device chosen as master does not identify as master.\n"); 
 }

 if (fd[1])
 {
   nuphase_read_register(dev,  REG_FIRMWARE_VER, fwver, SLAVE); 
   if (fwver[1])
   {
     fprintf(stderr,"WARNING! The device chosen as slave does not identify as slave.\n"); 
   }
 }


  if (nuphase_reset(dev, NP_RESET_COUNTERS)) 
  {
    fprintf(stderr,"Unable to reset device... "); 
    nuphase_close(dev); 
    return 0; 
  }

  return dev; 

}

void nuphase_set_board_id(nuphase_dev_t * d, uint8_t id, nuphase_which_board_t which)
{
  if (id <= board_id_counter) board_id_counter = id+1; 
  d->board_id[which] = id; 
}

uint8_t nuphase_get_board_id(const nuphase_dev_t * d, nuphase_which_board_t which) 
{
  return d->board_id[which]; 
}

void nuphase_set_readout_number_offset(nuphase_dev_t * d, uint64_t offset) 
{
  d->readout_number_offset = offset; 
}


void nuphase_set_buffer_length(nuphase_dev_t * d, uint16_t length)
{
  USING(d); //definitely do not want to change this mid readout 
  d->buffer_length = length; 
  DONE(d); 
}

uint16_t nuphase_get_bufferlength(const nuphase_dev_t * d) 
{
  return d->buffer_length; 
}


int nuphase_fwinfo(nuphase_dev_t * d, nuphase_fwinfo_t * info, nuphase_which_board_t which)
{

  //we need to read 5 registers, so 15 xfers 
  int ret = 0; 
  uint8_t version[NP_SPI_BYTES];
  uint8_t date[NP_SPI_BYTES];
  uint8_t dna_low[NP_SPI_BYTES]; 
  uint8_t dna_mid[NP_SPI_BYTES]; 
  uint8_t dna_hi[NP_SPI_BYTES]; 

  USING(d); 
  ret+=append_read_register(d, which,REG_FIRMWARE_VER, version); 
  ret+=append_read_register(d, which,REG_FIRMWARE_DATE, date); 
  ret+=append_read_register(d, which,REG_CHIPID_LOW, dna_low); 
  ret+=append_read_register(d, which,REG_CHIPID_MID, dna_mid); 
  ret+=append_read_register(d, which,REG_CHIPID_HI, dna_hi); 

  ret+=buffer_send(d, which); 
  DONE(d); 
  info->ver.major = version[3] >>4 ; 
  info->ver.minor = version[3] & 0x0f; 
  info->ver.master = version[1] & 1; 
  info->date.day = date[3] & 0xff; 
  info->date.month = date[2] & 0xf; 
  info->date.year = (date[2] >> 4) + (date[1] << 4); 

  //TODO check this logic. not sure endianness is correct
  uint64_t dna_low_big =  ((uint64_t) dna_low[3]) | ((uint64_t) dna_low[2]) << 8 | ((uint64_t) dna_low[1]) << 16;
  uint64_t dna_mid_big =  ((uint64_t) dna_mid[3]) | ((uint64_t) dna_mid[2]) << 8 | ((uint64_t) dna_mid[1]) << 16;
  uint64_t dna_hi_big =   ((uint64_t) dna_hi[3])  | ((uint64_t)dna_hi[2]) << 8 ;
  info->dna =  (dna_low_big & 0xffffff) | ( (dna_mid_big & 0xffffff) << 24) | ( (dna_hi_big & 0xffff) << 48); 

  return ret; 
}


int nuphase_close(nuphase_dev_t * d) 
{
  int ret = 0; 
  nuphase_cancel_wait(d); 
  int ibd;
  USING(d); 

  //clear the buffers! 
  for (ibd = 0; ibd < NBD(d); ibd++)
  {
    ret+= buffer_send(d, ibd); 
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

  if (d->gpio_pin)
  {
   ret += 256*bbb_gpio_close(d->gpio_pin,0); 
  }


  if (d->fd[1])
  {
    ret += flock(d->fd[1], LOCK_UN); 
    ret += 4*close(d->fd[1]); 
  }
  ret += 8*flock(d->fd[0], LOCK_UN); 
  ret += 16*close(d->fd[0]); 



  free(d); 
  return ret; 
}

void nuphase_cancel_wait(nuphase_dev_t *d) 
{
  d->cancel_wait = 1;  
}

int nuphase_wait(nuphase_dev_t * d, nuphase_buffer_mask_t * ready_buffers, float timeout, nuphase_which_board_t which) 
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
    if (d->enable_locking) pthread_mutex_unlock(&d->wait_mut);   //unlock the mutex
    return EAGAIN; 
  }



  nuphase_buffer_mask_t something = 0; 
  struct timespec start; 
  if (timeout >0) clock_gettime(CLOCK_MONOTONIC, &start); 

  float waited = 0; 
  // keep trying until we either get something, are cancelled, or exceed our timeout (if we have a timeout) 
  while(!something && (timeout <= 0 || waited < timeout))
  {

      something = nuphase_check_buffers(d,&d->hardware_next,which); 

      if (d->cancel_wait) break; 
      if (!something)
      {

        if(d->poll_interval)
        {
          usleep(d->poll_interval); 
        }
        else
        {
          sched_yield();
        }

        if (timeout >0)
        {
          struct timespec now; 
          clock_gettime(CLOCK_REALTIME, &now); 
          waited = (now.tv_sec - start.tv_nsec) * 1e-9f  * (now.tv_nsec - start.tv_nsec); 
        }
      }
  }
  int interrupted = d->cancel_wait; //were we interrupted? 

  if (ready_buffers) *ready_buffers = something;  //save to ready
  d->cancel_wait = 0;  //clear the wait
  if (d->enable_locking) pthread_mutex_unlock(&d->wait_mut);   //unlock the mutex
  return interrupted ? EINTR : 0; 


}



nuphase_buffer_mask_t nuphase_check_buffers(nuphase_dev_t * d, uint8_t * next, nuphase_which_board_t which) 
{

  uint8_t result[NP_SPI_BYTES]; 
  nuphase_buffer_mask_t mask; 
  int ret = 0; 

  USING(d); 
  ret+=append_read_register(d, which,REG_STATUS, result); 
  ret+= buffer_send(d,which); 
  DONE(d); 
  mask  = result[3] &  BUF_MASK; // only keep lower 4 bits.
  if (next) *next = (result[2] >> 4) & 0x3; 
  return mask; 
}


int nuphase_set_pretrigger(nuphase_dev_t * d, uint8_t pretrigger)
{
  uint8_t pretrigger_buf[] = { REG_PRETRIGGER, 0, 0, pretrigger & 0x7}; 
  int ret = synchronized_command(d, pretrigger_buf,0,0,0); 
  if (!ret) d->pretrigger = pretrigger; 
  return ret; 
}

uint8_t nuphase_get_pretrigger(const nuphase_dev_t * d)
{
  return d->pretrigger; 
}



int nuphase_set_channel_mask(nuphase_dev_t * d, uint8_t mask) 

{
    uint8_t channel_mask_buf_master[NP_SPI_BYTES]= { REG_CHANNEL_MASK, 0, 0, mask & 0xff}; 

    USING(d); 
    int written = do_write(d->fd[MASTER], channel_mask_buf_master); 
    DONE(d); 

    return written != NP_SPI_BYTES; 
}


uint16_t nuphase_get_channel_mask(nuphase_dev_t* d) 
{
  uint16_t mask = 0; 
  uint8_t buf_master[NP_SPI_BYTES], buf_slave[NP_SPI_BYTES]; 

  nuphase_read_register(d, REG_CHANNEL_MASK, buf_master, MASTER); 
  mask = buf_master[3]; 

  if (d->fd[SLAVE]) 
  {
    nuphase_read_register(d, REG_CHANNEL_MASK, buf_slave, SLAVE); 
    mask = mask |  ( buf_slave[3] << 8); 
  }

  return mask; 
}


int nuphase_set_trigger_mask(nuphase_dev_t * d, uint16_t mask)
{
  uint8_t trigger_mask_buf[]= { REG_TRIGGER_MASK, 0, (mask >> 8) & 0xff, mask & 0xff}; 
  USING(d); 
  int written = do_write(d->fd[MASTER], trigger_mask_buf); 
  DONE(d); 
  return written !=4; 
}


uint16_t nuphase_get_trigger_mask(nuphase_dev_t *d) 
{
  uint8_t buf[NP_SPI_BYTES]; 
  uint16_t mask; 
  nuphase_read_register(d, REG_TRIGGER_MASK, buf, MASTER); 
  mask = buf[3]; 
  mask = mask | (buf[2] << 8); 
  return mask; 
}


int nuphase_set_thresholds(nuphase_dev_t *d, const uint32_t * trigger_thresholds, uint16_t dont) 
{
  uint8_t thresholds_buf[NP_NUM_BEAMS][NP_SPI_BYTES]; 
  USING(d); 
  int i; 
  int ret = 0; 
  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    if (dont & (i << i)) continue; 
    int threshold = trigger_thresholds[i] < d->min_threshold ? d->min_threshold: trigger_thresholds[i]; 
    threshold = threshold <= 0xfffff ?  threshold : 0xfffff;
    thresholds_buf[i][0]= REG_THRESHOLDS+i ;
    thresholds_buf[i][1]= (threshold >> 16 ) & 0xf;
    thresholds_buf[i][2]= (threshold >> 8) & 0xff; 
    thresholds_buf[i][3]= threshold & 0xff;
    ret += buffer_append (d,MASTER,thresholds_buf[i],0); 
  }
    
  ret += buffer_send(d,MASTER); 
  DONE(d); 

  return ret; 
}

int nuphase_get_thresholds(nuphase_dev_t *d, uint32_t * thresholds) 
{
  uint8_t thresholds_buf[NP_NUM_BEAMS][NP_SPI_BYTES]; 
  int i; 
  int ret = 0; 
  USING(d); 
  for (i = 0; i < NP_NUM_BEAMS; i++)
  {
    ret+= append_read_register(d, MASTER, REG_THRESHOLDS+i, thresholds_buf[i]); 
  }
  ret += buffer_send(d,MASTER); 
  DONE(d); 

  if (ret) 
  {
    memset(thresholds, 0,NP_NUM_BEAMS * sizeof(*thresholds)); 
  }
  else
  {
    for (i = 0; i < NP_NUM_BEAMS; i++)
    {
      thresholds[i] = thresholds_buf[i][3] & 0xff; 
      thresholds[i] = thresholds[i] |  ( (thresholds_buf[i][2] & 0xff) << 8); 
      thresholds[i] = thresholds[i] |  ( (thresholds_buf[i][1] & 0xf) << 16); 
    }
  }

  return ret; 
}

#ifdef __arm__ 

uint8_t reverse_bits(uint8_t in) 
{
  uint32_t input = in;
  uint32_t output;
  __asm__("rbit %0, %1\n" : "=r"(output) : "r"(input));
  return output >> 24;
}
#else
uint8_t reverse_bits(uint8_t in) 
{
  uint8_t out = 0; 
  int i; 
  for (i = 0; i < 8; i++) 
  {
    if (in & (1 << i))
    {
      out = out | ( 1 << (7-i)); 
    }
  }

  return out; 
}
#endif

void reverse_buf_bits(uint8_t * buf) 
{
  reverse_bits(buf[3]); 
  reverse_bits(buf[2]); 
  reverse_bits(buf[1]); 

}




int nuphase_set_attenuation(nuphase_dev_t * d, const uint8_t * attenuation_master, const uint8_t * attenuation_slave)
{
  int ret = 0; 
  if (attenuation_master)
  {
    uint8_t attenuation_012[NP_SPI_BYTES] = { REG_ATTEN_012, attenuation_master[2], attenuation_master[1], attenuation_master[0] }; 
    uint8_t attenuation_345[NP_SPI_BYTES] = { REG_ATTEN_345, attenuation_master[5], attenuation_master[4], attenuation_master[3] };
    uint8_t attenuation_067[NP_SPI_BYTES] = { REG_ATTEN_67, 0x0, attenuation_master[7], attenuation_master[6] }; 
    reverse_buf_bits(attenuation_012);
    reverse_buf_bits(attenuation_345);
    reverse_buf_bits(attenuation_067);

    USING(d); 
    ret += buffer_append(d,MASTER, attenuation_012,0); 
    ret += buffer_append(d,MASTER, attenuation_345,0); 
    ret += buffer_append(d,MASTER, attenuation_067,0); 
    ret += buffer_send(d,MASTER); 
    DONE(d); 
  }

  if (attenuation_slave && d->fd[SLAVE])
  {
    uint8_t attenuation_012[NP_SPI_BYTES] = { REG_ATTEN_012, attenuation_slave[2], attenuation_slave[1], attenuation_slave[0] }; 
    uint8_t attenuation_345[NP_SPI_BYTES] = { REG_ATTEN_345, attenuation_slave[5], attenuation_slave[4], attenuation_slave[3] };
    uint8_t attenuation_067[NP_SPI_BYTES] = { REG_ATTEN_67, 0x0, attenuation_slave[7], attenuation_slave[6] }; 
    reverse_buf_bits(attenuation_012);
    reverse_buf_bits(attenuation_345);
    reverse_buf_bits(attenuation_067);
    USING(d); 
    ret += buffer_append(d,SLAVE, attenuation_012,0); 
    ret += buffer_append(d,SLAVE, attenuation_345,0); 
    ret += buffer_append(d,SLAVE, attenuation_067,0); 
    ret += buffer_send(d,SLAVE); 
    DONE(d); 
  }

  USING(d); 
  ret += synchronized_command(d, buf_apply_attenuation, 0,0,0); 
  DONE(d); 

  return ret; 
}

int nuphase_get_attenuation(nuphase_dev_t * d, uint8_t * attenuation_master, uint8_t * attenuation_slave)
{
  int ret = 0; 
  uint8_t attenuation_012[NP_SPI_BYTES];
  uint8_t attenuation_345[NP_SPI_BYTES];
  uint8_t attenuation_067[NP_SPI_BYTES];


  if (attenuation_master) 
  {
    USING(d); 
    ret += append_read_register(d,MASTER,REG_ATTEN_012, attenuation_012); 
    ret += append_read_register(d,MASTER,REG_ATTEN_345, attenuation_345); 
    ret += append_read_register(d,MASTER,REG_ATTEN_67 , attenuation_067); 
    ret += buffer_send(d,MASTER); 
    DONE(d)


    reverse_buf_bits(attenuation_012);
    reverse_buf_bits(attenuation_345);
    reverse_buf_bits(attenuation_067);

    if (!ret) 
    {
      attenuation_master[0] =  attenuation_012[3]; 
      attenuation_master[1] =  attenuation_012[2]; 
      attenuation_master[2] =  attenuation_012[1]; 
      attenuation_master[3] =  attenuation_345[3]; 
      attenuation_master[4] =  attenuation_345[2]; 
      attenuation_master[5] =  attenuation_345[1]; 
      attenuation_master[6] =  attenuation_067[3]; 
      attenuation_master[7] =  attenuation_067[2]; 
    }
  }

  if (!ret && attenuation_slave && d->fd[SLAVE])
  {
    USING(d); 
    ret += append_read_register(d,MASTER,REG_ATTEN_012, attenuation_012); 
    ret += append_read_register(d,MASTER,REG_ATTEN_345, attenuation_345); 
    ret += append_read_register(d,MASTER,REG_ATTEN_67 , attenuation_067); 
    ret += buffer_send(d,MASTER); 
    DONE(d)

    reverse_buf_bits(attenuation_012);
    reverse_buf_bits(attenuation_345);
    reverse_buf_bits(attenuation_067);

    if (!ret) 
    {
      attenuation_slave[0] =  attenuation_012[3]; 
      attenuation_slave[1] =  attenuation_012[2]; 
      attenuation_slave[2] =  attenuation_012[1]; 
      attenuation_slave[3] =  attenuation_345[3]; 
      attenuation_slave[4] =  attenuation_345[2]; 
      attenuation_slave[5] =  attenuation_345[1]; 
      attenuation_slave[6] =  attenuation_067[3]; 
      attenuation_slave[7] =  attenuation_067[2]; 
    }


  }

  return ret; 
}


int nuphase_set_trigger_enables(nuphase_dev_t * d, nuphase_trigger_enable_t enables, nuphase_which_board_t w) 
{
  uint8_t trigger_enable_buf[NP_SPI_BYTES] = {REG_TRIG_ENABLE, 0, enables.enable_beam8 | (enables.enable_beam4a << 1) | (enables.enable_beam4b << 2), enables.enable_beamforming}; 

//  printf("Setting trigger enables: [0x%x 0x%x 0x%x 0x%x]\n", trigger_enable_buf[0], trigger_enable_buf[1], trigger_enable_buf[2], trigger_enable_buf[3]); 
  USING(d); 
  int written = do_write(d->fd[w], trigger_enable_buf); 
  DONE(d); 
  return written != NP_SPI_BYTES ; 
}

nuphase_trigger_enable_t nuphase_get_trigger_enables(nuphase_dev_t * d, nuphase_which_board_t w) 
{
  uint8_t trigger_enable_buf[NP_SPI_BYTES]; 
  nuphase_read_register(d,REG_TRIG_ENABLE, trigger_enable_buf, w); 
//  printf("Got trigger enables: [0x%x 0x%x 0x%x 0x%x]\n", trigger_enable_buf[0], trigger_enable_buf[1], trigger_enable_buf[2], trigger_enable_buf[3]); 
  nuphase_trigger_enable_t ans; 
  ans.enable_beamforming = trigger_enable_buf[3] & 1; 
  ans.enable_beam8 = trigger_enable_buf[2] & 1; 
  ans.enable_beam4a = (trigger_enable_buf[2] >> 1) & 1; 
  ans.enable_beam4b = (trigger_enable_buf[2] >> 2) & 1; 

  return ans; 
}

int nuphase_phased_trigger_readout(nuphase_dev_t * d, int phased) 
{
    uint8_t trigger_buf[NP_SPI_BYTES] = {REG_PHASED_TRIGGER, 0, 0, phased & 1}; 
    USING(d); 
    if (d->fd[1]) do_write(d->fd[SLAVE], trigger_buf); 
    do_write(d->fd[MASTER], trigger_buf); 
    DONE(d); 

    return 0; 

}

int set_trigger_holdoff(nuphase_dev_t * d, uint16_t trigger_holdoff)
{
  uint8_t trigger_holdoff_buf[NP_SPI_BYTES] = {REG_TRIG_HOLDOFF, 0, (trigger_holdoff >> 8) & 0xf, trigger_holdoff &0xff}; 
  USING(d); 
  int written = do_write(d->fd[MASTER], trigger_holdoff_buf); 
  DONE(d); 
  return (written != NP_SPI_BYTES) ;
}

uint16_t nuphase_get_trigger_holdoff(nuphase_dev_t *d) 
{
  uint8_t trigger_holdoff_buf[NP_SPI_BYTES]; 
  nuphase_read_register(d, REG_TRIG_HOLDOFF, trigger_holdoff_buf, MASTER); 
  return trigger_holdoff_buf[3] |  ( trigger_holdoff_buf[2] << 8); 
}




//indirection! 
int nuphase_wait_for_and_read_multiple_events(nuphase_dev_t * d, 
                                      nuphase_header_t (*headers)[NP_NUM_BUFFER], 
                                      nuphase_event_t  (*events)[NP_NUM_BUFFER])  
{
  nuphase_buffer_mask_t mask; ; 
  nuphase_wait(d,&mask,-1,MASTER); 
  if (mask) 
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
  int ibd; 


  for (iibuf = 0; iibuf < __builtin_popcount(mask); iibuf++)
  {

    ibuf = d->next_read_buffer; 
    hd[iout]->sync_problem = 0; 
    for (ibd = 0; ibd < NBD(d); ibd++)
    {

      //we are not reading this event right now
      if ( (mask & (1 << ibuf)) == 0)
      {
        fprintf(stderr,"Sync issue? d->next_read_buffer=%d, mask=0x%x, hardware next: %d\n", d->next_read_buffer, mask, d->hardware_next); 
        easy_break_point(); 
        d->next_read_buffer =  __builtin_ctz(mask); //pick the lowest buffer to read next 
        ibuf = d->next_read_buffer; 
        break; 
      }

      clock_gettime(CLOCK_REALTIME, &now); 

      //grab the metadata 
      //set the buffer 
      USING(d); 
      if (ibd == 0) 
      {
        d->event_counter++; 
        d->next_read_buffer = (d->next_read_buffer + 1) %NP_NUM_BUFFER; 
      }
      CHK(buffer_append(d, ibd,  buf_buffer[ibuf],0)) 
      d->current_buf[ibd] = ibuf; 

      /**Grab metadata! */ 
      //switch to register mode  

      //we will pretend like we are bigendian so we can just call be64toh on the u64
      CHK(append_read_register(d,ibd,REG_EVENT_COUNTER_LOW, (uint8_t*) &event_counter[0])) 
      CHK(append_read_register(d,ibd,REG_EVENT_COUNTER_HIGH, (uint8_t*) &event_counter[1])) 
      CHK(append_read_register(d,ibd,REG_TRIG_COUNTER_LOW, (uint8_t*) &trig_counter[0])) 
      CHK(append_read_register(d,ibd,REG_TRIG_COUNTER_HIGH,(uint8_t*)  &trig_counter[1])) 
      CHK(append_read_register(d,ibd,REG_TRIG_TIME_LOW,(uint8_t*)  &trig_time[0])) 
      CHK(append_read_register(d,ibd,REG_TRIG_TIME_HIGH,(uint8_t*)  &trig_time[1])) 
      CHK(append_read_register(d,ibd,REG_DEADTIME, (uint8_t*) &deadtime)) 
      CHK(append_read_register(d,ibd,REG_TRIG_INFO, (uint8_t*) &tinfo)) 

      if (ibd == 0)  // these don't make sense for a slave 
      {
        CHK(append_read_register(d,ibd,REG_TRIG_MASKS,(uint8_t*) &tmask)) 
        for(ibeam = 0; ibeam < NP_NUM_BEAMS; ibeam++)
        {
          CHK(append_read_register(d,ibd, REG_BEAM_POWER+ibeam, (uint8_t*)  &hd[iout]->beam_power[ibeam]))
        }
      }

     //flush the metadata .  we could get slightly faster throughput by storing metadata 
      //read locations for each buffer and not flushing.
      // If it ends up mattering, I'll change it. 
      
      CHK(buffer_send(d,ibd)); 
      DONE(d);//yield  

#ifdef DEBUG_PRINTOUTS
      printf("Raw tinfo: %x\n", tinfo) ;
      printf("Raw tmask: %x\n", tmask) ;
#endif 
      
      // check the event counter
      event_counter[0] = be32toh(event_counter[0]) & 0xffffff; 
      event_counter[1] = be32toh(event_counter[1]) & 0xffffff; 
      trig_counter[0] = be32toh(trig_counter[0]) & 0xffffff; 
      trig_counter[1] = be32toh(trig_counter[1]) & 0xffffff; 
      trig_time[0] = be32toh(trig_time[0]) & 0xffffff; 
      trig_time[1] = be32toh(trig_time[1]) & 0xffffff; 

#ifdef DEBUG_PRINTOUTS
      printf("Raw event_counter: %x %x\n", event_counter[0], event_counter[1]) ;
      printf("Raw trig_counter: %x %x\n", trig_counter[0], trig_counter[1]) ;
      printf("Raw trig_time: %x %x \n", trig_time[0], trig_time[1]) ;
#endif 


      uint64_t big_event_counter = event_counter[0] + (event_counter[1] << 24); 

      if (d->event_counter !=  big_event_counter)
      {
        fprintf(stderr,"Event counter mismatch!!! (bd: %s sw: %"PRIu64", hw: %"PRIu64")\n", ibd ? "SLAVE" : "MASTER" , d->event_counter, big_event_counter); 
        easy_break_point(); 
      }

      //now fill in header data 
      tinfo = be32toh(tinfo); 
      tmask = be32toh(tmask); 

      uint8_t hwbuf =  (tinfo >> 22) & 0x3; 
      if ( hwbuf  != ibuf)
      {
          fprintf(stderr,"Buffer number mismatch!!! (bd %d sw: %u, hw: %u)\n", ibd, ibuf, hwbuf ); 
          easy_break_point(); 
          hd[iout]->sync_problem |= 1; 
      }
     

      hd[iout]->readout_time[ibd] = now.tv_sec; 
      hd[iout]->readout_time_ns[ibd] = now.tv_nsec; 
      hd[iout]->trig_time[ibd] =trig_time[0] + (trig_time[1] << 24); 
      hd[iout]->channel_read_mask[ibd] = d->channel_read_mask[ibd]; 
      hd[iout]->deadtime[ibd] = be32toh(deadtime) & 0xffffff; 
      hd[iout]->board_id[ibd] = d->board_id[ibd]; 
 
      //values that we only save for the master
      if (ibd == 0)
      {
        hd[iout]->event_number = d->readout_number_offset + big_event_counter; 
        hd[iout]->trig_number = trig_counter[0] + (trig_counter[1] << 24); 
        hd[iout]->buffer_length = d->buffer_length; 
        hd[iout]->pretrigger_samples = d->pretrigger* 8 * 16; //TODO define these constants somewhere
        hd[iout]->approx_trigger_time= d->start_time.tv_sec + hd[iout]->trig_time[ibd] / BOARD_CLOCK_HZ; 
        hd[iout]->approx_trigger_time_nsecs = d->start_time.tv_nsec + (hd[iout]->trig_time[ibd] % BOARD_CLOCK_HZ) *(1.e9 / BOARD_CLOCK_HZ); 
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

  
        hd[iout]->buffer_number = hwbuf; 
        hd[iout]->channel_overflow = 0; //TODO not implemented yet 
        hd[iout]->buffer_mask = mask; //this is the current buffer mask
        hd[iout]->trig_type = (tinfo >> 15) & 0x3; 
        hd[iout]->calpulser = (tinfo >> 21) & 0x1; 
        hd[iout]->channel_mask = (tmask >> 15) & 0xff; 


        //event stuff
        ev[iout]->buffer_length = d->buffer_length; 
        ev[iout]->event_number = hd[iout]->event_number; 
 
      }
      else //do some checks
      {



        if (hd[iout]->trig_number != trig_counter[0] + (trig_counter[1] << 24))
        {
          fprintf(stderr,"trig number mismatch between master and slave %"PRIu64" vs %"PRIu64"!\n", hd[iout]->trig_number, (uint64_t) trig_counter[0] + (trig_counter[1] <<24)); 
          hd[iout]->sync_problem |= 2; 
        }

        if (abs(hd[iout]->trig_time[ibd] -  hd[iout]->trig_time[0]) > 2)
        {
          static unsigned nprinted = 0; 

          if (nprinted < 10) 
          {
            nprinted++; 
            fprintf(stderr,"Trig times differ by more than 2 clock cycles between boards! (printing %d more times) \n", 10-nprinted); 
          }

          hd[iout]->sync_problem |= 4; 
        }

        if (hwbuf != hd[iout]->buffer_number)
        {

          fprintf(stderr,"Buffer numbers differ between boards!\n"); 
          hd[iout]->sync_problem |=8; 
        }
      }


      ev[iout]->board_id[ibd] = d->board_id[ibd]; 

      //now start to read the data 
      for (ichan = 0; ichan < NP_NUM_CHAN; ichan++)
      {
        if ( d->channel_read_mask[ibd] & ( 1 << ichan) )
        {
          USING(d);  
          if (d->current_mode[ibd] != MODE_WAVEFORMS)
          {
            CHK(buffer_append(d,ibd, buf_mode[MODE_WAVEFORMS],0))
            d->current_mode[ibd] = MODE_WAVEFORMS; 
          }

          if (d->current_buf[ibd] != ibuf)
          {
            CHK(buffer_append(d,ibd, buf_buffer[ibuf],0))
          }

          if (d->channel_read_mask[ibd] & (1 << ichan)) //TODO is this backwards?!??? 
          {
            CHK(buffer_append(d,ibd, buf_channel[ichan],0)) 
            CHK(loop_over_chunks_half_duplex(d,ibd, d->buffer_length / (NP_SPI_BYTES * NP_NUM_CHUNK),1, &ev[iout]->data[ibd][ichan][0]))
          }
          DONE(d); 
        }
        else
        {
          memset(&ev[iout]->data[ibd][ichan][0], 0 , d->buffer_length); 
        }
      }


      //zero out things that don't make sense if there is no slave
      if (NBD(d) < 2) 
      {
        hd[iout]->readout_time[1] = 0; 
        hd[iout]->readout_time_ns[1] = 0; 
        hd[iout]->trig_time[1] = 0; 
        hd[iout]->deadtime[1] = 0; 
        hd[iout]->board_id[1] = 0; 
        memset(ev[iout]->data[1],0, NP_NUM_CHAN * NP_MAX_WAVEFORM_LENGTH); 

      }

      USING(d); 
      CHK(buffer_send(d,ibd)); 
      DONE(d); 
    }
    mark_buffers_done(d, 1 << ibuf); 
    iout++; 

  }


  the_end:
  //TODO add some printout here in case of falure/ 
  //also, need to unlock mutex if it's locked 

  return ret; 
}


int nuphase_clear_buffer(nuphase_dev_t *d, nuphase_buffer_mask_t mask) 
{
  USING(d); 
  int ret = mark_buffers_done(d,mask); 
  DONE(d); 
  return ret; 
}

int nuphase_write(nuphase_dev_t *d, const uint8_t* buffer)
{
  int written = 0; 
  USING(d); 
  written = do_write(d->fd[0], buffer); 
  if (d->fd[1]) 
  written += do_write(d->fd[1], buffer); 
  DONE(d); 
  return written == d->fd[1] ? 2 * NP_SPI_BYTES : NP_SPI_BYTES ? 0 : -1; 
}

int nuphase_read(nuphase_dev_t *d,uint8_t* buffer, nuphase_which_board_t which)
{
  int got = 0; 
  USING(d); 
  got = do_read(d->fd[which], buffer); 
  DONE(d); 
  return got == NP_SPI_BYTES ? 0 : -1; 
}



int nuphase_read_status(nuphase_dev_t *d, nuphase_status_t * st, nuphase_which_board_t which) 
{
  //TODO: fill in deadtime when I figure out how. 
  int i; 
  int ret = 0; 
  struct timespec now; 
  uint8_t scaler_registers[N_SCALER_REGISTERS][NP_SPI_BYTES]; 

  uint8_t latched_pps[2][NP_SPI_BYTES]; 

  st->board_id = d->board_id[which]; 

  USING(d); 
  ret+=buffer_append(d, which,buf_mode[MODE_REGISTER],0); 
  d->current_mode[which] = MODE_REGISTER; 
  ret+=buffer_append(d,which, buf_update_scalers,0); 

  for (i = 0; i < N_SCALER_REGISTERS; i++) 
  {
    ret+=buffer_append(d,which, buf_pick_scaler[i],0); 
    ret+=append_read_register(d,which, REG_SCALER_READ, scaler_registers[i]); 
  }

  //also add the latched time stamp 
  
  ret += append_read_register(d,which, REG_LATCHED_PPS_LOW, latched_pps[0]); 
  ret += append_read_register(d,which, REG_LATCHED_PPS_HIGH, latched_pps[1]); 
  
  clock_gettime(CLOCK_REALTIME, &now); 
  ret+= buffer_send(d,which); 
  DONE(d); 

  ret+= nuphase_get_thresholds(d, &st->trigger_thresholds[0]); 

  if (ret) return ret; 
  st->deadtime = 0; //TODO 


  for (i = 0; i < N_SCALER_REGISTERS; i++) 
  {
    uint16_t first = ((uint16_t)scaler_registers[i][3])  |  (((uint16_t) scaler_registers[i][2] & 0xf ) << 8); 
    uint16_t second =((uint16_t)(scaler_registers[i][2] >> 4)) |  (((uint16_t) scaler_registers[i][1] ) << 4); 
//    printf("%d %u %u\n", i, first, second); 

    int which_scaler = i / ((1 + NP_NUM_BEAMS)/2); 
    int which_channel = i % (( 1 + NP_NUM_BEAMS)/2); 

    if (which_channel == 0) 
    {
      st->global_scalers[which_scaler] = first; 
      st->beam_scalers[which_scaler][0]= second; 
    }
    else
    {
      st->beam_scalers[which_scaler][2*which_channel-1]= first; 
      st->beam_scalers[which_scaler][2*which_channel] = second; 
    }
  }

  st->latched_pps_time = latched_pps[0][3]; 
  st->latched_pps_time |= ((uint64_t) latched_pps[0][2]) << 8; 
  st->latched_pps_time |= ((uint64_t) latched_pps[0][1]) << 16; 
  st->latched_pps_time |= ((uint64_t) latched_pps[1][3]) << 24; 
  st->latched_pps_time |= ((uint64_t) latched_pps[1][2]) << 32; 
  st->latched_pps_time |= ((uint64_t) latched_pps[1][1]) << 40; 


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
int nuphase_reset(nuphase_dev_t * d, nuphase_reset_t reset_type)
{
  
//  const nuphase_config_t * cfgs[NP_MAX_BOARDS]; 
//  cfgs[0] = c; 
//  cfgs[1] = cslave; 

  int wrote; 
  int ibd;

  // We start by tickling the right reset register
  // if we are doing a global, almost global or ADC reset. 
  // We need to verify that these sleep delays are good.
  
  if (reset_type == NP_RESET_GLOBAL) 
  {
    if (synchronized_command(d,buf_reset_all,0,0,0))
    {
        return 1;
    }
 
    fprintf(stderr,"Full reset...\n"); 
    //we need to sleep for a while. how about 20 seconds? 
    //TODO add check on register 8 
    sleep(20); 
    fprintf(stderr,"...done\n"); 
  }
  else if (reset_type == NP_RESET_ALMOST_GLOBAL)
  {
    for (ibd = 0; ibd < NBD(d); ibd++)
    {
      wrote = do_write(d->fd[ibd], buf_reset_almost_all); 

      if (wrote != NP_SPI_BYTES) 
      {
        return 1;
      }
    }

    fprintf(stderr,"Almost full reset...\n"); 
    //we need to sleep for a while. how about 20 seconds? 
    sleep(20); 
    fprintf(stderr,"...done\n"); 
  }

  /* after all resets (if applicable), we want to restart the event counter 
   * and, if any of the stronger resets were applied, apply the calibration. 
   *
   * The order of operations is: 
   *
   *  - turn off the phased trigger
   *  - clear all the buffers 
   *  - if necessary, do the calibration 
   *  - reset the event / trig time counters (and save the time to try to match it up later) 
   *  
   
   **/


  if (nuphase_phased_trigger_readout(d,0)) 
  {
        fprintf(stderr, "Unable to turn off readout. Aborting reset\n"); 
        return 1; 
  }

  for (ibd = 0; ibd < NBD(d); ibd++)
  {
    //clear all buffers, and reset to zero
    wrote = do_write (d->fd[ibd], buf_clear[0xf]); 
    wrote += do_write (d->fd[ibd], buf_reset_buf); 

    if (wrote != 2*NP_SPI_BYTES) 
    {
        fprintf(stderr, "Unable to clear buffers. Aborting reset\n"); 
        return 1; 
    }
  }


  d->next_read_buffer = 0; 

  //do the calibration, if necessary 
  // THIS IS NOT FULLY WORKING YET FOR NOW WE WILL USE ERIC'S align_adcs.py 
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
  if (reset_type >= NP_RESET_CALIBRATE) 
  {
    int happy = 0; 
    int misery = 0; 
    wrote = NP_SPI_BYTES; 

    //temporarily set the buffer length to the maximum 
    uint16_t old_buf_length = d->buffer_length; 
    d->buffer_length = 1024; 

    //we need to turn off the phased trigger to not overwhelm ARA 
    nuphase_trigger_enable_t old_enables = nuphase_get_trigger_enables(d, MASTER); 
    nuphase_trigger_enable_t tmp_enables; 
    memcpy(&tmp_enables, &old_enables, sizeof(old_enables)); 
    tmp_enables.enable_beamforming = 0; 
    nuphase_set_trigger_enables(d, tmp_enables, MASTER); 

    //release the calpulser 
    nuphase_calpulse(d, 3); 

    while (!happy) 
    {
      if (misery++ > 0) 
      {
        if (misery> 3) 
        {
          fprintf(stderr,"Misery now at %d\n", misery); 
        }

        if (misery > MAX_MISERY) 
        {
          fprintf(stderr,"Maximum misery reached. We can't take it anymore. Giving up on ADC alignment and not bothering to configure.\n"); 
          break; 
        }

        if (d->fd[1]) //synchronize the buf_adc_clk_rst
        {
          if(synchronized_command(d, buf_adc_clk_rst, 0,0,0))  
          {
            fprintf(stderr,"problem sending buf_adc_clk_rst\n"); 
            continue;
          }
          sleep(1); 
        }
        else
        {
          wrote = do_write(d->fd[0], buf_adc_clk_rst); 
          if ( wrote != NP_SPI_BYTES) 
          {
            fprintf(stderr,"When adc_clk_rst, expected %d got %d\n", NP_SPI_BYTES, wrote);  
            continue; 
          }
        }
      }


      nuphase_buffer_mask_t mask; 
      nuphase_sw_trigger(d); 
      nuphase_wait(d,&mask,1,MASTER); 
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
      uint16_t max_i[2][NP_NUM_CHAN];
      memset(max_i,0,sizeof(max_i)); 

      //loop through and find where the maxes are
      int ichan, isamp; 

      for (ibd = 0; ibd < NBD(d); ibd++)
      {
        for (ichan = 0; ichan <NP_NUM_CHAN; ichan++)
        {
          if ( ((1<<ichan) & d->channel_read_mask[ibd])  == 0) continue; 

          uint8_t max_v = 0; 
          for (isamp = 0; isamp < NP_MAX_WAVEFORM_LENGTH; isamp++)
          {
            if ( d->calib_ev.data[ibd][ichan][isamp] > max_v)
            {
              max_v = d->calib_ev.data[ibd][ichan][isamp]; 
              max_i[ibd][ichan] = isamp; 
            }
          }
          
          printf("max_i,max_v for bd %d chan %d is %d,%d\n", ibd,ichan,max_i[ibd][ichan],max_v); 

          if (max_i[ibd][ichan] < min_max_i) min_max_i = max_i[ibd][ichan]; 
          if (max_i[ibd][ichan] > max_max_i)  max_max_i = max_i[ibd][ichan]; 
          if (max_v < min_max_v)  min_max_v = max_v; 
        }
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
      for (ibd = 0; ibd < NBD(d); ibd++)
      {
        for (iadc = 0; iadc < NP_NUM_CHAN/2; iadc++)
        {
          if (((1 << 2*iadc) & d->channel_read_mask[ibd])  == 0) continue; 

          uint8_t delay  = (max_i[ibd][2*iadc] + max_i[ibd][2*iadc+1]- 2*min_max_i)/2; 
          //TODO!!! 
          
          if (delay > 0) 
          {
            uint8_t buf[NP_SPI_BYTES] = {REG_ADC_DELAYS + iadc, 0, (delay & 0xf) | (1 << 4) , (delay & 0xf)  | (1 << 4) }; 
            wrote = do_write(d->fd[ibd], buf); 
            if (wrote < NP_SPI_BYTES) 
            {
              fprintf(stderr,"Should have written %d but wrote %d\n", NP_SPI_BYTES, wrote); 
              continue;//why not? 
            }
          }
       }
      }

     //yay
      happy=1; 
    }

    d->buffer_length = old_buf_length; 
    nuphase_calpulse(d, 0); 

    // reclear the buffers 
    for (ibd = 0; ibd < NBD(d); ibd++) 
    {
      do_write(d->fd[ibd], buf_clear[0xf]); 
    }

    nuphase_set_trigger_enables(d, old_enables, MASTER); 
    if (!happy) return -1; 
  }

  //then reset the counters, measuring the time before and after 
   
   struct timespec tbefore; 
   struct timespec tafter; 
   if (NBD(d) > 1) 
   {
     clock_gettime(CLOCK_REALTIME,&tbefore); 
     if (synchronized_command(d, buf_reset_counter,0,0,0))
     {
        fprintf(stderr, "Unable to reset counters. Aborting reset\n"); 
        return 1; 
     }
     clock_gettime(CLOCK_REALTIME,&tafter); 
   }
   else
   {
     clock_gettime(CLOCK_REALTIME,&tbefore); 
     wrote = do_write(d->fd[0], buf_reset_counter); 
     clock_gettime(CLOCK_REALTIME,&tafter); 
     if (wrote != NP_SPI_BYTES) 
     {
        fprintf(stderr, "Unable to reset counters. Aborting reset\n"); 
        return 1; 
     }
   }
    

    //take average for the start time
    d->start_time = avg_time(tbefore,tafter); 

   int ret  = 0; 

   return ret ; 
}

int nuphase_set_spi_clock(nuphase_dev_t *d, unsigned clock) 
{

  int ibd;
  d->spi_clock = clock*1000000; 
  USING(d); 
  for (ibd = 0; ibd < NBD(d); ibd++)
  {
    ioctl(d->fd[ibd], SPI_IOC_WR_MAX_SPEED_HZ, &d->spi_clock); 
  }
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


int nuphase_get_trigger_output(nuphase_dev_t *d, nuphase_trigger_output_config_t * config) 
{

  uint8_t cfg_buf[NP_SPI_BYTES]; 
  int ret = nuphase_read_register(d, REG_TRIGOUT_CONFIG, cfg_buf, MASTER); 
  
  config->width = cfg_buf[2]; 
  config->enable = cfg_buf[3] & 1; 
  config->polarity = (cfg_buf[3] >> 1)  & 1; 
  config->send_1Hz = (cfg_buf[3] >> 2)  & 1; 

  return ret; 
}


int nuphase_configure_trigger_output(nuphase_dev_t *d, nuphase_trigger_output_config_t config) 
{
  uint8_t cfg_buf[NP_SPI_BYTES] = { REG_TRIGOUT_CONFIG,0, config.width,
                                    (config.enable & 1 ) | ((config.polarity & 1) <<1) 
                                     | ((config.send_1Hz & 1) << 2) 
                                  }; 

  USING(d); 
  int written = do_write(d->fd[MASTER], cfg_buf); 
  DONE(d); 
  return written != NP_SPI_BYTES; 
}
int nuphase_configure_ext_trigger_in(nuphase_dev_t * d, nuphase_ext_input_config_t config) 
{
  uint8_t cfg_buf[NP_SPI_BYTES] = { REG_EXT_INPUT_CONFIG, 
                                   config.gate_width >> 8,
                                   config.gate_width & 8,
                                   (config.use_as_trigger & 1) | ((config.gate_enable & 1)<<1)} ; 
  USING(d); 
  int written = do_write(d->fd[MASTER], cfg_buf); 
  DONE(d); 
  return written != NP_SPI_BYTES; 
}

/** get the external trigger config */ 
int nuphase_get_ext_trigger_in(nuphase_dev_t * d, nuphase_ext_input_config_t * config) 
{
  uint8_t cfg_buf[NP_SPI_ENABLE]; 
  int ret = nuphase_read_register(d, REG_EXT_INPUT_CONFIG, cfg_buf, MASTER); 

  config->use_as_trigger = cfg_buf[3] & 1; 
  config->gate_enable = (cfg_buf[3] >> 1)  & 1; 
  config->gate_width = cfg_buf[2] | (cfg_buf[1] << 8); 
  return ret; 
}


int nuphase_enable_verification_mode(nuphase_dev_t * d, int mode) 
{
  uint8_t buf[NP_SPI_BYTES] = { REG_VERIFICATION_MODE,0,0, mode & 1}; 
  USING(d); 
  int written = do_write(d->fd[MASTER], buf); 
  DONE(d); 
  return written != NP_SPI_BYTES; 
}

int nuphase_query_verification_mode(nuphase_dev_t * d) 
{
  uint8_t buf[NP_SPI_BYTES]; 
  int ret = nuphase_read_register(d,REG_VERIFICATION_MODE,buf,MASTER); 
  if (ret) return -1; 
  return buf[3] & 1; 
}

int nuphase_set_poll_interval(nuphase_dev_t * d, uint16_t interval)
{
  d->poll_interval = interval ;
  return 0; 
}


