#include "bbb_gpio.h" 
#include <stdio.h> 
#include <fcntl.h> 
#include <stdlib.h>
#include <unistd.h> 
#include <string.h>
#include <sys/file.h> 
#include <errno.h>

struct bbb_gpio_pin
{
  int value_fd; 
  int dir_fd; 
  int num; 
}; 

const char * gpio_path = "/sys/class/gpio/gpio%d";  
const char * gpio_value_path = "/sys/class/gpio/gpio%d/value";  
const char * gpio_dir_path = "/sys/class/gpio/gpio%d/direction";  
const char * gpio_export_path = "/sys/class/gpio/export";  
const char * gpio_unexport_path = "/sys/class/gpio/unexport"; 


const char * dirstr[]  = { "in","out"}; 

bbb_gpio_pin_t *  bbb_gpio_open(int gpio_pin) 
{
  

  // First check to see if it's already been exported 
  char buf[512];  
  sprintf(buf,gpio_path, gpio_pin); 

  if (access(buf, F_OK)) //try exporting it if it's not already exported
  {

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
  }


  sprintf(buf, gpio_value_path, gpio_pin); 
  int fd = open(buf, O_RDWR);

  if ( fd < 0) 
  {
    fprintf(stderr,"Could not open %s\n", buf); 
    return 0;
  }

  sprintf(buf, gpio_dir_path, gpio_pin); 
  int dir_fd = open(buf, O_RDWR); 
  if ( dir_fd < 0 ) 
  {
    fprintf(stderr,"Could not open %s\n", buf); 
    close(fd); 
    return 0;

  }

  //lock access to prevent more than one program talking to it at a time
  // this only works with other programs using flock... so the python based stuff I assume won't work (unless we modify it to flock) 
  if (flock(fd, LOCK_EX | LOCK_NB) || flock(dir_fd, LOCK_EX | LOCK_NB)) 
  {
    fprintf(stderr,"Could not obtain exclusive access to GPIO %d", gpio_pin); 
    close(dir_fd); 
    close(fd); 

    return 0; 
  }

  bbb_gpio_pin_t * pin = malloc(sizeof(bbb_gpio_pin_t)); 

  pin->value_fd = fd; 
  pin->dir_fd = dir_fd; 
  pin->num = gpio_pin; 

  return pin; 
}


int bbb_gpio_pin_number(const bbb_gpio_pin_t * pin) 
{
  return pin->num; 
}



int bbb_gpio_get(bbb_gpio_pin_t * pin) 
{
  char st; 
  if ( read(pin->value_fd, &st, 1) <0)
  {
    fprintf(stderr,"Problem reading from pin %d. errno: %d\n", pin->num, errno); 
    return -1; 
  }

  return  (int) (st - '0'); 
}



int bbb_gpio_set(bbb_gpio_pin_t * pin, int state) 
{

  int ret = 0; 

  if (state) 
  {
    ret = write(pin->dir_fd, "high", strlen("high")); 
  }
  else
  {
    ret = write(pin->dir_fd, "low", strlen("low")); 
  }

  if ( ret <0)
  {
    fprintf(stderr,"Problem writing to pin %d. errno: %d\n", pin->num, errno); 
    return -1; 
  }

  return 0; 
}


int bbb_gpio_set_direction(bbb_gpio_pin_t * pin, bbb_gpio_direction_t dir)
{
  if ( write(pin->dir_fd, dirstr[dir], strlen(dirstr[dir])) < 0)
  {
    fprintf(stderr,"Trouble changing direction to \"%s\" for GPIO %d\n",dirstr[dir], pin->num); 
    return -1; 
  }

  return 0; 
}

bbb_gpio_direction_t bbb_gpio_get_direction(bbb_gpio_pin_t * pin) 
{
  char letter; 
  //just read the first letter
  if (read (pin->dir_fd, &letter, 1) < 0) 
  {
    fprintf(stderr,"Trouble getting direction from GPIO %d\n", pin->num); 
    return -1; 
  }

  return letter == 'o'  ? BBB_OUT : BBB_IN; 
}

int bbb_gpio_close(bbb_gpio_pin_t * pin, int unexport)
{
  //unlock 
  flock(pin->dir_fd, LOCK_UN); 
  flock(pin->value_fd, LOCK_UN); 
  //close file descriptor 
  close(pin->value_fd); 
  close(pin->dir_fd); 

  //unexport
  if (unexport) 
  {
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
  }

  //free memory
  free(pin); 

  return 0; 
}


