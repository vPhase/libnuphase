#include "nuphasehk.h"
#include "bbb_ain.h" 
#include "bbb_gpio.h" 

#include <time.h> 
#include <stdio.h> 
#include <sys/statvfs.h> 
#include <stdlib.h>
#include <sys/sysinfo.h> 
#include <termios.h> 
#include <unistd.h> 
#include <string.h> 
#include <fcntl.h> 

#ifdef WITH_CURL 
#include <curl/curl.h> 
#endif



#define MASTER_TEMP_AIN 0
#define SLAVE_TEMP_AIN  2
#define MASTER_POWER_GPIO 46
#define SLAVE_POWER_GPIO 47
#define COMM_GPIO 60
#define DOWNHOLE_GPIO 66


//---------------------------------------------------
// HK initialization, offering option to initialize with the given settings
//   Also, keep track if we have been init to init on first call and
//   prevent initializing more than once 
//-------------------------------------------------
static int already_init_hk = 0; 

//---------------------------------------------
// Global HK settings 
//---------------------------------------------
static nuphase_hk_settings_t cfg = {.asps_serial_device = 0, .asps_address = 0} ; 


int nuphase_hk_init(const nuphase_hk_settings_t * settings) 
{

  if (already_init_hk) 
  {
    fprintf(stderr,"Cannot init hk more than once.\n"); 
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

  already_init_hk = 1; 
  return 0; 

}

static int gpios_are_setup = 0;

//---------------------------------------------
//   device handles for the GPIO pins 
//---------------------------------------------
static bbb_gpio_pin_t * master_fpga_ctl = 0; 
static bbb_gpio_pin_t * slave_fpga_ctl = 0; 
static bbb_gpio_pin_t * comm_ctl = 0; 
static bbb_gpio_pin_t * downhole_power_ctl = 0; 


/** GPIO Setup
 *
 *  This just exports them
 *  
 **/ 
static int setup_gpio() 
{
  // take control of the gpio's 
  int ret = 0; 

  master_fpga_ctl = bbb_gpio_open(MASTER_POWER_GPIO);
  if (!master_fpga_ctl) ret+=1;  

  slave_fpga_ctl = bbb_gpio_open(SLAVE_POWER_GPIO); 
  if (!slave_fpga_ctl) ret+=2;  

  comm_ctl = bbb_gpio_open(COMM_GPIO); 
  if (!comm_ctl) ret+=4; 

  downhole_power_ctl = bbb_gpio_open(DOWNHOLE_GPIO); 
  if (!downhole_power_ctl) ret+=8; 

  gpios_are_setup = 1; 
  return ret;
}


//------------------------------------------------------
//default settings 
//------------------------------------------------------
void nuphase_hk_settings_init(nuphase_hk_settings_t * settings) 
{
  settings->asps_serial_device = "/dev/ttyUSB0"; 
  settings->asps_address = "asps-daq";  // this can be defined, for example, in /etc/hosts
}

/* parse the json string for the current. very dumb */
static int parse_string_for_current (const char * buf) 
{

  char * start = strstr(buf, "{\"pid\":["); 
  if (!start) 
  {
    return -1; 
  }
      
  int current ;

  if (sscanf(start,"{\"pid\":[%d", &current) < 1) 
  {
    return -1;  //found less than 1 token 
  }

  return current; 
}



//------------------------------------------------------
// now comes http handling code, for communicating with
// the ASPS-DAQ via http. We use the cURL "easy" API 
// to save the web page into a buffer (http_buf), and then 
// parse it by looking four our magic parse-friendly strings
//---------------------------------------------------------

#ifdef WITH_CURL
static CURL * curl = 0; //cURL handle
static char * http_buf = 0;  //the http buffer 
static size_t http_buf_pos = 0;  //our position within the http buffer
static size_t http_buf_size = 0;  //the current size of the buffer 

//this is our cURL callback that copies into our buffer
static size_t save_http(char * ptr, size_t size, size_t nmemb, void * user) 
{
  (void) user; 

  if (!http_buf)  // we haven't allocated a buffer yet. let's do it. minimum of 16K or whatever cURL passes us, +1 for null byte
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
//  printf("memcpy(%x, %x, %u)\n", http_buf + http_buf_pos, ptr, size*nmemb); 
  memcpy (http_buf + http_buf_pos, ptr, size*nmemb); 
  http_buf_pos += size * nmemb; 
  http_buf[http_buf_pos] = 0; // set the null byte
//  printf("save_http called, buf is :%s\n", http_buf); 

  return size * nmemb;  //if we don't return the size given, cURL gets angry
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
    asprintf(&update_url, "http://%s/parse.html", cfg.asps_address); 
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

static char * http_set_buf = 0; 
/// this sets the power state using http and a GET
static int http_set (const nuphase_asps_power_state_t  state) 
{
  if (!http_set_buf ) http_set_buf = malloc(strlen(cfg.asps_address)+64); 
  sprintf(http_set_buf, "http://%s?0=%d&1=%d&2=%d&3=%d&4=%d" , cfg.asps_address, !!(state & 1), !!(state & 2), !!(state & 4), !!(state & 8), !!(state & 16)); 

  //init cURL if not init
  if (!curl) curl = curl_easy_init(); 

  //set the URL 
  curl_easy_setopt(curl, CURLOPT_URL, http_set_buf); 

  //we don't need to download the body afterwards 
  curl_easy_setopt(curl, CURLOPT_NOBODY,1); 

  return curl_easy_perform(curl); 
}

/// this sets the power state using http and a GET
static char * heater_url = 0; 
static int http_set_heater (int current) 
{
  if (!heater_url ) 
  {
    asprintf(&heater_url, "http://%s/heater.html", cfg.asps_address); 
  }

  char postbuf[32]; 
  sprintf(postbuf,"heater=%d", current); 

  //init cURL if not init
  if (!curl) curl = curl_easy_init(); 

  //set the URL 
  curl_easy_setopt(curl, CURLOPT_URL, heater_url); 

  curl_easy_setopt(curl, CURLOPT_POSTFIELDS,postbuf); 

  //might as well save the output. We could check to make sure it was successful I suppose... 
  curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, save_http); 

  //reset http buf position
  http_buf_pos = 0; 


  return curl_easy_perform(curl); 
}

// this will load the power via a GET reequest 
static int http_get_heater() 
{
  if (!heater_url ) 
  {
    asprintf(&heater_url, "http://%s/heater.html", cfg.asps_address); 
  }

  //init our cURL handle if it hasn't been already
  if (!curl) curl = curl_easy_init(); 
 
  // set the url 
  curl_easy_setopt( curl, CURLOPT_URL, heater_url); 

  // make sure we download
  curl_easy_setopt(curl, CURLOPT_HTTPGET,1); 

  // set the call back 
  curl_easy_setopt( curl, CURLOPT_WRITEFUNCTION, save_http); 

  int nfails = 0;
  while (nfails++ < 5)
  {
    // reset the http buf position 
    http_buf_pos = 0; 

    curl_easy_perform(curl); 

    int current = parse_string_for_current(http_buf); 
    if (current >=0 ) return current; 

    fprintf(stderr,"Couldn't find current in string \"%s\"\n", http_buf); 
  }

  fprintf(stderr,"Too many fails in set heater :(\n"); 
  return -1; 

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

  if (heater_url) 
  {
    free(heater_url); 
  }

  return 0; 
}

#else 
static int http_set(const nuphase_asps_power_state_t state) 
{
  (void) state; 
  fprintf(stderr,"Compiled without cURL support\n"); 
  return 1;
}

static int http_update(nuphase_hk_t * hk) 
{
  (void) hk; 
  fprintf(stderr,"Compiled without cURL support\n"); 
  return 1; 
}

static int http_set_heater(int current) 
{
  (void) current; 
  fprintf(stderr,"Compiled without cURL support\n"); 
  return 1;
}

static int http_get_heater() 
{
  fprintf(stderr,"Compiled without cURL support\n"); 
  return -1; 
}



#endif


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

  if (serial_fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", cfg.asps_serial_device); 
    exit(1); 
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


  const char  query_string[] = "hkbin\r"; 
  binary_hk_data data; 
  int nfails = 0; 
  if (!serial_fd) serial_init(); 

  tcflush(serial_fd, TCIOFLUSH); 

  //If the ASPS-DAQ gets rebooted while we're querying it, we are going to get
  //a bunch of junk.  That is pretty unlikely, but it could happen . So we'll
  //try a few times
  while (nfails++ < 5) 
  {
    write(serial_fd,query_string, sizeof(query_string)-1); 
    //looks like CmdArduino echoes everything 
    char garbage = '0'; 
    while (garbage != '\n') 
    {
      read(serial_fd, &garbage,1); 
    }

    read(serial_fd, &data, sizeof(data)); 
    tcflush(serial_fd, TCIOFLUSH); 

    if ( data.magic_start != 0xe110 || data.magic_end != 0xef0f) 
    {
      fprintf(stderr,"Didn't get right magic bytes. Got: %x %x Will try again.\n", data.magic_start, data.magic_end); 
      write(serial_fd,"\r",1); 
      usleep(1000); 
      tcflush(serial_fd, TCIOFLUSH); 
      
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

//------------------------------------------------------------------------------------------
// set the power state using our new ctlmask command
// TODO: If the ASPS-DAQ restarts in the middle of this, 
//       it probably won't be able to process our message properly.
//       
//       We could flush before and read after to make sure that there is nothing unexpected. 
//
//------------------------------------------------------------------------------------------
static int serial_set(const nuphase_asps_method_t state) 
{
  char buf[128]; 
  //note... this mask isn't in hex right now because the ASPS-DAQ isn't expecting in hex. 
  int len = sprintf(buf,"\rctlmask %u\r", state); 
  if (!serial_fd) serial_init(); 
  int written =  write(serial_fd, buf, len); 
  tcdrain(serial_fd); 
  return written != len; 
}

static int serial_set_heater(int current) 
{
  char buf[128]; 
  int len = sprintf(buf,"\rheaterCurrent %u\r", current); 
  if (!serial_fd) serial_init(); 
  int written =  write(serial_fd, buf, len); 
  tcdrain(serial_fd); 
  return written != len; 
}

static int serial_get_heater() 
{
  char buf[256]; 
  char writebuf[64]; 
  int len = sprintf(writebuf,"heaterLine\r"); 
  if (!serial_fd) serial_init(); 
  tcflush(serial_fd, TCIOFLUSH); 
  int nfails = 0; 
  while ( nfails++ < 5) 
  {
    write(serial_fd, writebuf, len); 


    //throw away the first line
    char garbage = '0'; 
    int pos = 0; 
    while (garbage != '\n') read(serial_fd, &garbage,1); 

    //now read in a line 
    do 
    {
      read(serial_fd, &buf[pos++],1); 
    }
    while ( buf[pos-1] != '\n' && pos < 256); 

    
    int current = parse_string_for_current(buf); 


    if ( current >=0) return current; 
    else
    {
      fprintf(stderr,"Could not read current in string: \"%s\"\n", buf); 
    }
  }

  fprintf(stderr,"Too many fails in get heater :(\n"); 
  return -1; 
}

//---------------------------------------------------
// Read in the GPIO state
// -------------------------------------------------
static nuphase_gpio_power_state_t query_gpio_state() 
{

  if (!gpios_are_setup) setup_gpio(); 
  nuphase_gpio_power_state_t state = 0; 

  //master is on as an input, I think
  if (!master_fpga_ctl || bbb_gpio_get(master_fpga_ctl) )
  {
    state = state | NP_FPGA_POWER_MASTER; 
  }
  
  //slave is on as an input, I think
  if (!slave_fpga_ctl || bbb_gpio_get(slave_fpga_ctl) )
  {
    state = state | NP_FPGA_POWER_SLAVE; 
  }


  if (comm_ctl && bbb_gpio_get(comm_ctl) == 0)
  {
    state = state | NP_SPI_ENABLE; 
  }

  if (downhole_power_ctl && bbb_gpio_get(downhole_power_ctl) == 1)
  {
    state = state | NP_DOWNHOLE_POWER; 
  }

  return state; 
}


//--------------------------------------
//temperature probe conversion 
//-------------------------------------
static float mV_to_C(float val_mV) 
{
  return (1858.3-2*val_mV)  * 0.08569; 
}



//----------------------------------------
//The main hk update method 
//----------------------------------------
int nuphase_hk(nuphase_hk_t * hk, nuphase_asps_method_t method ) 
{

  if (!already_init_hk) nuphase_hk_init(0); 

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
  hk->temp_master = mV_to_C(bbb_ain_mV(MASTER_TEMP_AIN)); 
  hk->temp_slave = mV_to_C(bbb_ain_mV(SLAVE_TEMP_AIN)); 

  /* figure out the disk space  and memory*/ 
  statvfs("/", &fs); 
  hk->disk_space_kB = fs.f_bsize * (fs.f_bavail >> 10) ; 
  sysinfo(&mem); 
  hk->free_mem_kB = (mem.freeram * mem.mem_unit) >> 10 ;   //this doesn't properly take into account of cache / buffers, which would require parsing /proc/meminfo I think 

  /* check our gpio state */ 
  hk->gpio_state = query_gpio_state()  ; 

  //get the time
  clock_gettime(CLOCK_REALTIME_COARSE, &now); 
  hk->unixTime = now.tv_sec; 
  hk->unixTimeMillisecs = now.tv_nsec / (1000000); 
  
  return 0; 

}


//-----------------------------------------
// The ASPS power state delegator 
//-----------------------------------------
int nuphase_set_asps_power_state(nuphase_asps_power_state_t st, nuphase_asps_method_t method) 
{

  if (!already_init_hk) nuphase_hk_init(0); 

  if (method == NP_ASPS_HTTP) 
    return http_set(st); 
  else
    return serial_set(st); 
}

//-----------------------------------------
//  GPIO State 
//-----------------------------------------
int nuphase_set_gpio_power_state ( nuphase_gpio_power_state_t state) 
{
  if (!already_init_hk) nuphase_hk_init(0); 
  if (! gpios_are_setup) setup_gpio(); 


  int ret = 0; 
  ret += !master_fpga_ctl || bbb_gpio_set( master_fpga_ctl, (state & NP_FPGA_POWER_MASTER)); 
  ret += !slave_fpga_ctl || bbb_gpio_set( slave_fpga_ctl, (state & NP_FPGA_POWER_SLAVE)); 

  //this one is active low
  ret += !comm_ctl || bbb_gpio_set( comm_ctl, !(state & NP_SPI_ENABLE) ); 

  ret += !downhole_power_ctl || bbb_gpio_set( downhole_power_ctl, (state & NP_DOWNHOLE_POWER) ); 

  return ret; 
}


///////////////////////////////////////////////////////////
//  Dispatcher for getting the current 
///////////////////////////////////////////////////////////
int nuphase_get_heater_current(nuphase_asps_method_t method) 
{
  if (!already_init_hk) nuphase_hk_init(0); 

  if (method == NP_ASPS_HTTP) 
  {
    return http_get_heater(); 
  }
  else return serial_get_heater(); 
}

///////////////////////////////////////////////////////////
//  Dispatcher for setting the current 
//  /////////////////////////////////////////////////////////////////
int nuphase_set_heater_current(int current, nuphase_asps_method_t method) 
{
  if (!already_init_hk) nuphase_hk_init(0); 

  if (method == NP_ASPS_HTTP) 
  {
    return http_set_heater(current); 
  }
  else return serial_set_heater(current); 

}





//-----------------------------------------
//    deinit
//-----------------------------------------
__attribute__((destructor)) 
static void nuphase_hk_destroy() 
{
  //do NOT unexport any of these!
  if (master_fpga_ctl) bbb_gpio_close(master_fpga_ctl,0); 
  if (slave_fpga_ctl) bbb_gpio_close(slave_fpga_ctl,0); 
  if (comm_ctl) bbb_gpio_close(comm_ctl,0); 
  if (downhole_power_ctl) bbb_gpio_close(downhole_power_ctl,0);  
}





