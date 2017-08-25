#include "bbb_gpio.h" 
#include <stdio.h> 
#include <fcntl.h> 
#include <stdlib.h>
#include <unistd.h> 
#include <string.h>
#include <errno.h>

struct bbb_gpio_pin
{
  int fd; 
  int num; 
  bbb_gpio_direction_t dir; 
}; 

const char * gpio_path = "/sys/class/gpio/gpio%d";  
const char * gpio_value_path = "/sys/class/gpio/gpio%d/value";  
const char * gpio_dir_path = "/sys/class/gpio/gpio%d/direction";  
const char * gpio_export_path = "/sys/class/gpio/export";  
const char * gpio_unexport_path = "/sys/class/gpio/unexport"; 


const char * dirstr[]  = { "in","out"}; 

bbb_gpio_pin_t *  bbb_gpio_open(int gpio_pin, bbb_gpio_direction_t dir) 
{
  

  // First check to see if it's already been exported 
  char buf[512];  
  sprintf(buf,gpio_path, gpio_pin); 

  if (! access(buf, F_OK))
  {
    fprintf(stderr,"%s already exists... does someone else already have this GPIO open?. I support exclusive access only. \n", buf); 
    return 0; 
  }

  int export_fd = open(gpio_export_path,O_WRONLY); 

  if (export_fd < 0) 
  {
    fprintf(stderr,"Could not write %s. insufficient permissions?\n", gpio_export_path); 
    return 0; 
  }

  int len = sprintf(buf, "%d", gpio_pin); 
  if (write(export_fd, buf, len) < 0 ) 
  {
    fprintf(stderr, "Problem writing \"%s\" to %s: (ERR: %d)\n",buf, gpio_export_path, errno); 
    return 0; 
  }


  sprintf(buf, gpio_value_path, gpio_pin); 
  int fd = open(buf, O_RDWR);

  if ( fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", buf); 
    return 0;
  }


  bbb_gpio_pin_t * pin = malloc(sizeof(bbb_gpio_pin_t)); 

  pin->fd = fd; 
  pin->num = gpio_pin; 
  bbb_gpio_set_direction(pin,dir); 

  return pin; 
}


int bbb_gpio_pin_number(const bbb_gpio_pin_t * pin) 
{
  return pin->num; 
}

bbb_gpio_direction_t bbb_gpio_dir(const bbb_gpio_pin_t * pin) 
{
  return pin->dir; 
}

int bbb_gpio_get(bbb_gpio_pin_t * pin) 
{
  if (pin->dir != BBB_IN) 
  {
    fprintf(stderr,"Trying to get pin %d which has the wrong direction\n", pin->num); 
    return -1; 
  }


  char st; 
  if ( read(pin->fd, &st, 1) <0)
  {
    fprintf(stderr,"Problem reading from pin %d. errno: %d\n", pin->num, errno); 
    return -1; 
  }

  return  (int) (st - '0'); 
}



int bbb_gpio_set(bbb_gpio_pin_t * pin, int state) 
{
  if (pin->dir != BBB_OUT) 
  {
    fprintf(stderr,"Trying to set pin %d which has the wrong direction\n", pin->num); 
    return -1; 
  }

  char st= '0' + !!state; 

  if ( write(pin->fd, &st, 1) <0)
  {
    fprintf(stderr,"Problem writing to pin %d. errno: %d\n", pin->num, errno); 
    return -1; 
  }

  return 0; 
}


int bbb_gpio_set_direction(bbb_gpio_pin_t * pin, bbb_gpio_direction_t dir)
{

  char buf[512]; 
  sprintf(buf, gpio_dir_path, pin->num); 
  int dir_fd = open(buf,O_WRONLY); 

  if (dir_fd < 0) 
  {
    fprintf(stderr,"Could not open for %s to set pin %d direction.",buf, pin->num); 
    return -1; 
  }

  if ( write(dir_fd, dirstr[dir], strlen(dirstr[dir])) < 0)
  {
    fprintf(stderr,"Trouble writing \"%s\" to %s\n",dirstr[dir], buf); 
    return -1; 
  }

  pin->dir = dir; 

  return 0; 
}


int bbb_gpio_close(bbb_gpio_pin_t * pin)
{
  //close file descriptor 
  close(pin->fd); 

  //unexport
  int unexport_fd = open(gpio_unexport_path,O_WRONLY); 

  if (unexport_fd < 0) 
  {
    fprintf(stderr,"Could not write %s. insufficient permissions?\n", gpio_unexport_path); 
    return -1; 
  }

  char buf[512]; 
  int len = sprintf(buf, "%d", pin->num); 
  if (write(unexport_fd, buf, len) < 0 ) 
  {
    fprintf(stderr, "Problem writing \"%s\" to %s: (ERR: %d)\n", buf, gpio_unexport_path, errno); 
    return -1; 
  }

  //free memory
  free(pin); 

  return 0; 
}


