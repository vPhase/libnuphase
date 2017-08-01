#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h> 
#include <sys/file.h>
#include <sys/ioctl.h>


// write_word spidev byte0 byte1 byte2 byte3 
int main(int nargs, char ** args) 
{
  uint8_t bytes[4]; 
  int i;


  if (nargs < 6) 
  {
    printf("Usage: write_word spidev b0 b1 b2 b3\n"); 
    return 1; 
  }

  //no interrupt, no locking, default config, 
  int dev = open(args[1],O_RDWR); 

  ioctl(dev, SPI_IOC_WR_MODE,0); 
  ioctl(dev, SPI_IOC_WR_MAX_SPEED_HZ, 10000000); 

  for (i = 0; i < 4; i++)
  {
    bytes[i] = strtol(args[i+2], 0, 16); 
  }

  printf("Sending:(%x,%x,%x,%x)\n", bytes[0], bytes[1], bytes[2], bytes[3]); 
  write(dev, bytes, 4); 

  close(dev); 

  return 0; 
}
