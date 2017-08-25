#ifndef bbb_gpio_h 
#define bbb_gpio_h 

/**GPIO helper library for BeagleBoneBlack 
 *
 * Cosmin Deaconu
 * cozzyd@kicp.uchicago.edu
 */ 
 

/* Opaque device handle */
typedef struct bbb_gpio_pin bbb_gpio_pin_t; 

/** pin direction */ 
typedef enum bbb_gpio_direction
{
  BBB_IN, 
  BBB_OUT, 
} bbb_gpio_direction_t; 



/** open the given pin GPIO . Will allocate memory and return an opaque pointer if successful, 0 otherwise.*/ 
bbb_gpio_pin_t * bbb_gpio_open(int gpio_pin, bbb_gpio_direction_t dir); 


/** Set the GPIO direction. Returns 0 on success, -1 if something went wrong. Does NOT check to see if it needs
 * to change direction or not.  */ 
int bbb_gpio_set_direction(bbb_gpio_pin_t * pin, bbb_gpio_direction_t dir); 


/** Sets the pin value to the given state. returns 0 on success, -1 if something went wrong.. 
 * Make sure you set the direction to out first! 
 */
int bbb_gpio_set(bbb_gpio_pin_t * pin, int state); 

/** Gets the bin value (0 or 1). Returns -1 if something went wrong. Make sure you set the direction first! */ 
int bbb_gpio_get(bbb_gpio_pin_t * pin); 

/** Close the gpio pin. Return 0 on success */ 
int bbb_gpio_close(bbb_gpio_pin_t * pin); 

int bbb_gpio_pin_number(const bbb_gpio_pin_t * pin); 

/** Gets the direction. Could be wrong if it's been changed from underneath us
 *  by some process that doesn't care that we're already using it */ 
bbb_gpio_direction_t bbb_gpio_dir(const bbb_gpio_pin_t * pin); 


#endif
