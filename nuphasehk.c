//for asprintf 
#define _GNU_SOURCE


#include "nuphasehk.h"
#include "bbb_ain.h" 
#include "bbb_gpio.h" 

#include <time.h> 
#include <curl/curl.h> 
#include <stdio.h> 
#include <sys/statvfs.h> 
#include <stdlib.h>
#include <sys/sysinfo.h> 
#include <termios.h> 
#include <unistd.h> 
#include <string.h> 
#include <fcntl.h> 


//---------------------------------------------
//   device handles for the FPGA pins 
//---------------------------------------------
static bbb_gpio_pin_t * master_fpga_ctl = 0; 
static bbb_gpio_pin_t * slave_fpga_ctl = 0; 

//---------------------------------------------
// Global HK settings 
//---------------------------------------------
static nuphase_hk_settings_t cfg; 


//---------------------------------------------------
// HK initializiation, offering option to initialize with the given settings
//   Also, keep track if we have been init to init on first call and
//   prevent initializing more than once 
//-------------------------------------------------
static int already_init = 0; 
int nuphase_hk_init(const nuphase_hk_settings_t * settings) 
{

  if (already_init) 
  {
    fprintf(stderr,"Cannot intialize hk more than once.\n"); 
    return 1;  
  }
  if (settings)
  {
    memcpy(&cfg, settings, sizeof(cfg)); 
  }
  else
  {
    nuphase_hk_settings_init(&cfg); 
  }

 //take control of the gpio's 
//  master_fpga_ctl  = bbio_gpio_open( cfg.master_fpga_power_gpio, BBB_IN); // should I turn it on or off... ? Leave it as input for now 
//  bbb_gpio_set_value ( master_fpga_ctl, 0);  //prepare to turn off 

//  slave_fpga_ctl  = bbio_gpio_open( cfg.master_fpga_power_gpio, BBB_IN); // should I turn it on or off... ? Leave it as input for now 
//  bbb_gpio_set_value (slave_fpga_ctl, 0);  //prepare to turn off 

  return 0;
}


//------------------------------------------------------
//default settings 
//------------------------------------------------------
void nuphase_hk_settings_init(nuphase_hk_settings_t * settings) 
{
  settings->master_temperature_ain = 0; 
  settings->slave_temperature_ain = 2; 
  settings->asps_serial_device = "/dev/ttyUSB0"; 
  settings->asps_address = "asps-daq";  // this can be defined, for example, in /etc/hosts
}

//------------------------------------------------------
// now comes http handling code, for communicating with
// the ASPS-DAQ via http. We use the cURL "easy" API 
// to save the web page into a buffer (http_buf), and then 
// parse it by looking four our magic parse-friendly strings
//---------------------------------------------------------

static CURL * curl = 0; //cURL handle
static char * http_buf = 0;  //the http buffer 
static size_t http_buf_pos = 0;  //our position within the http buffer
static size_t http_buf_size = 0;  //the current size of the buffer 

//this is our cURL callback that copies into our buffer
static size_t save_http(char * ptr, size_t size, size_t nmemb, void * user) 
{
  (void) user; 

  if (!http_buf_size)  // we haven't allocated a buffer yet. let's do it. minimum of 16K or whatever cURL passes us, +1 for null byte
  {
    http_buf_size = size*nmemb < 16 * 1024 ? 1+ 16 * 1024 : size*nmemb + 1; 
    http_buf = malloc(http_buf_size); 
    if (!http_buf) return 0; 
  }

  // see if we need a bigger buffer. if we do, allocate twice more what cURL gave us 
  else if (http_buf_size < http_buf_pos + size * nmemb + 1) 
  {
    http_buf_size = http_buf_pos + size*nmemb *2 + 1; 
    http_buf = realloc( http_buf, http_buf_size); 
    if (!http_buf) return 0; 
  }
  

  //copy into our buffer
  memcpy (http_buf + http_buf_pos, ptr, size*nmemb); 
  http_buf_pos += size * nmemb; 
  http_buf[http_buf_pos] = 0; // set the null byte

  return size + nmemb;  //if we don't return the size given, cURL gets angry
}

// this actually parses our buffer looking for our hk data 
static int parse_http (const char * httpbuf , nuphase_hk_t * hk) 
{

  char * start = strstr(httpbuf, "[=parsefriendly=]"); 
  if (!start) 
  {
    return -1; 
  }
      
  uint8_t on_state, fault_state;  //since we can't take the address of a bitfield
  // hhd is signed char, hu is unsigned short, hhx is hex unsigned char 
  if (sscanf(start,"[=parsefriendly=]%hhd;%hhd;%hu;%hu;%hu;%hu;%hu;%hhx;%hhx;", 
        &hk->temp_case, &hk->temp_asps_uc,
        &hk->current_master, &hk->current_slave,
        &hk->current_frontend, &hk->current_sbc,
        &hk->current_switch, &on_state,
        &fault_state) < 9)
  {
    return -1;  //found less than 9 tokens 
  }

  hk->on_state = on_state; 
  hk->fault_state = fault_state; 
  return 0; 
}

 // the url 
static char * update_url = 0;     

// this will load the power via a GET reequest 
static int http_update(nuphase_hk_t * hk) 
{

  //init our cURL handle if it hasn't been already
  if (!curl) curl = curl_easy_init(); 
 
  if (!update_url)   
  {
    asprintf(&update_url, "http://%s/", cfg.asps_address); 
  }

  // set the url 
  curl_easy_setopt( curl, CURLOPT_URL, update_url); 

  // reset the http buf position 
  http_buf_pos = 0; 

  // make sure we download
  curl_easy_setopt(curl, CURLOPT_HTTPGET,1); 

  // set the call back 
  curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, save_http); 

  // release the cURL 
  curl_easy_perform(curl); 

  //parse what we have wrought
  return parse_http(http_buf, hk); 
}

/// this sets the power state using http and a GET
static int http_set (const nuphase_asps_power_state_t  state) 
{
  static char * buf = 0; 
  if (!buf ) buf = malloc(strlen(cfg.asps_address)+64); 
  sprintf(buf, "http://%s?%d&%d&%d&%d&%d" , cfg.asps_address, state & 1, state & 2, state & 4, state & 8, state & 16); 

  //init cURL if not init
  if (!curl) curl = curl_easy_init(); 

  //set the URL 
  curl_easy_setopt(curl, CURLOPT_URL, buf); 

  //we don't need to download the body afterwards 
  curl_easy_setopt(curl, CURLOPT_NOBODY,1); 

  return curl_easy_perform(curl); 
}

//be a good citizen and deallocate our cURL stuff at the end 
__attribute__((destructor)) 
static int destroy_curl()
{
  if (curl) 
  {
    curl_easy_cleanup(curl); 
  }

  if (update_url) 
  {
    free(update_url); 
  }
  return 0; 
}




//-------------------------------------------------------
// Now comes the serial communication with the ASPSDAQ 
// we will use the serial stream in raw mode, so that we can
// process our nice binary data. 
//-------------------------------------------------------

//our serial file descriptor 
static int serial_fd = 0; 


// hopefully this is all we need 
static int serial_init() 
{
  struct termios t; //serial options
  serial_fd = open(cfg.asps_serial_device, O_RDWR ) ;  //open the file descriptor

  if (!serial_fd) 
  {
    fprintf(stderr,"Could not open %s\n", cfg.asps_serial_device); 
    return 1; 
  }

  //grab the current options
  tcgetattr(serial_fd, &t); 
  cfmakeraw(&t); //switch to char at a time processing. 
  cfsetspeed(&t, B38400);  //set baud rate to what the ASPS-DAQ expects 
  return tcsetattr(serial_fd, TCSANOW, &t); // set up the port properly 
}

//be a good citizen, even though the file descriptors will be closed regardless 
__attribute__ ((destructor)) 
static void serial_destroy()
{
  if (serial_fd) 
    close(serial_fd); 
}


typedef struct
{
  uint16_t magic_start;
  int8_t temp_case; 
  int8_t temp_uc; 
  uint16_t current_master; 
  uint16_t current_slave; 
  uint16_t current_frontend; 
  uint16_t current_sbc; 
  uint16_t current_switch; 
  uint8_t power_state; 
  uint8_t fault_state; 
  uint16_t magic_end; 
} binary_hk_data; 



//this uses our magic binary data transfer so we don't have to parse anything 
static int serial_update(nuphase_hk_t * hk) 
{

  const char * query_string = "binhk\n"; 
  binary_hk_data data; 
  int nfails = 0; 
  if (!serial_fd) serial_init(); 

  //If the ASPS-DAQ gets rebooted while we're querying it, we are going to get
  //a bunch of junk.  That is pretty unlikely, but it could happen . So we'll
  //try a few times
  
  while (nfails++ < 5) 
  {
    tcflush(serial_fd, TCIOFLUSH); 
    write(serial_fd,query_string, sizeof(query_string)-1); 
    read(serial_fd, &data, sizeof(data)); 

    if ( data.magic_start != 0xe110 || data.magic_end != 0xef0f) 
    {
      fprintf(stderr,"Didn't get right magic bytes. Will try again.\n"); 
    }
    else
    {
      hk->temp_case = data.temp_case; 
      hk->temp_asps_uc = data.temp_uc; 
      hk->current_master = data.current_master; 
      hk->current_slave = data.current_slave; 
      hk->current_frontend = data.current_frontend; 
      hk->current_sbc = data.current_sbc; 
      hk->current_switch = data.current_switch; 
      hk->on_state = data.power_state; 
      hk->fault_state = data.fault_state; 
      return 0;
    }
  }

  fprintf(stderr,"Too many fails. Giving up\n"); 
  return 1; 
}

// set the power state using our new ctlmask command
// TODO: If the ASPS-DAQ restarts in the middle of this, 
//       it probably won't be able to process our message properly.
//       
//       We could flush before and read after to make sure that there is nothing unexpected. 
//
static int serial_set(const nuphase_asps_method_t state) 
{
  char buf[128]; 
  //note... this mask isn't in hex right now because the ASPS-DAQ isn't expecting in hex. 
  int len = sprintf(buf,"ctlmask %u\n", state); 
  if (!serial_fd) serial_init(); 
  return write(serial_fd, buf, len) != len; 
}



//--------------------------------------
//temperature probe conversion 
//-------------------------------------
static float mV_to_C(float val_mV) 
{
  return (1858.3-val_mV)  * 0.08569; 
}



//----------------------------------------
//The main hk update method 
//----------------------------------------
int nuphase_hk(nuphase_hk_t * hk, nuphase_asps_method_t method ) 
{

  /* first the ASPS-DAQ bits, using the specified method. */

  struct timespec now; 
  struct statvfs fs; 
  struct sysinfo mem; 
  int ret = 0;
  if (method == NP_ASPS_HTTP) 
  {
    ret+= http_update(hk); 
  }
  else 
  {
    ret += serial_update(hk); 
  }

  /* now, read in our temperatures */ 
  hk->temp_master = mV_to_C(bbb_ain_mV(cfg.master_temperature_ain)); 
  hk->temp_slave = mV_to_C(bbb_ain_mV(cfg.slave_temperature_ain)); 

  /* figure out the disk space  and memory*/ 
  statvfs("/", &fs); 
  hk->disk_space_kB = fs.f_frsize * fs.f_bavail / 1024; 
  sysinfo(&mem); 
  hk->free_mem_kB = mem.freeram * mem.mem_unit / 1024;   //this doesn't properly take into account of cache / buffers, which would require parsing /proc/meminfo I think 

  /* check our gpio state */ 
  //TODO 

  //get the time
  clock_gettime(CLOCK_REALTIME_COARSE, &now); 
  hk->unixTime = now.tv_sec; 
  hk->unixTimeMillisecs = now.tv_nsec / (1000000); 
  
  return 0; 

}


int nuphsae_set_asps_power_state(nuphase_asps_power_state_t st, nuphase_asps_method_t method) 
{

  if (method == NP_ASPS_HTTP) 
    return http_set(st); 
  else
    return serial_set(st); 
}

int nuphase_set_fpga_power_state ( nuphase_fpga_power_state_t state) 
{

  //TODO 
  return 0; 

}


