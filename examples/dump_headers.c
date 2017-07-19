#include "nuphase.h" 
#include <stdio.h> 


int main(int nargs, char ** args) 
{

  if (nargs < 2) 
  {
    fprintf(stderr,"dump_headers headers.dat"); 
    return 1; 
  }



  FILE * f = fopen(args[1], "r"); 
  nuphase_header_t hd;

  while (!nuphase_header_read(f,&hd))
  {
    nuphase_print_header(stdout, &hd); 
  }

  fclose(f); 

  return 0; 





}
