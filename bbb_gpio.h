#ifndef bbb_gpio_h 
#define bbb_gpio_h 

/** 
 * \file bbb_gpio.h
 *
 * GPIO helper code for BeagleBoneBlack. This just messes with the sysfs filesystem. 
 *
 * Cosmin Deaconu
 * cozzyd@kicp.uchicago.edu
 */ 
 

/** Opaque device handle */
typedef struct bbb_gpio_pin bbb_gpio_pin_t; 

/** enum pin direction */ 
typedef enum bbb_gpio_direction
{
  BBB_IN, 
  BBB_OUT
} bbb_gpio_direction_t; 



/** open the given pin GPIO . Will allocate memory and return an opaque pointer if successful, 0 otherwise.
 * You can set the state and direction here too. The state is set before the direction. 
 * */ 
bbb_gpio_pin_t * bbb_gpio_open(int gpio_pin); 


/** Set the GPIO direction. Returns 0 on success, -1 if something went wrong. Does NOT check to see if it needs
 * to change direction or not.  */ 
int bbb_gpio_set_direction(bbb_gpio_pin_t * pin, bbb_gpio_direction_t dir); 


/** Sets the pin value to the given state. returns 0 on success, -1 if something went wrong.. 
 * ALSO SETS DIRECTION TO OUT 
 */
int bbb_gpio_set(bbb_gpio_pin_t * pin, int state); 

/** Gets the bin value (0 or 1). Returns -1 if something went wrong.  */ 
int bbb_gpio_get(bbb_gpio_pin_t * pin); 

/** Close the gpio pin. If unexport is true, it will become unexported (and I guess revert to an input). Return 0 on success */ 
int bbb_gpio_close(bbb_gpio_pin_t * pin, int unexport); 

/** returns the GPIO pin number associated with this pin */ 
int bbb_gpio_get_pin_number(const bbb_gpio_pin_t * pin); 

/** Gets the direction. */ 
bbb_gpio_direction_t bbb_gpio_get_direction(bbb_gpio_pin_t * pin); 


#endif
