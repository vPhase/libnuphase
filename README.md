libnuphase
----------

Hardware interface library for NuPhase Phased Array. 

This consists of shared libraries to support data taking and a bunch of
"examples" that are probably better thought of as tests to make sure things
work.


There are two shared libraries: 

  _libnuphase.so (+ nuphase.h):_

  Defines data types and has utilities for reading/writing/printing the data types. This is 
  useful not just on the DAQ machine but any machine that might interact with the types. 

  _libnuphasedaq.so (+ all others headers)_ 

  All the code for communication with the FPGA's, ASPS-DAQ, and other hardware. This is basically 
  only useful on the DAQ machine (a beaglebone-black) 


Quick commands: 

  - compile on DAQ: `make`

  - compile on non-DAQ: `make client` (only makes libnuphase.so, not libnuphasedaq.so) 

  - make doxygen documentation: `make doc`

  - install to /usr/local: `make install`

  - install elsewhere: `make install PREFIX=/somewhere/else`

  - client install: make install-client

Examples: 

  See examples directory. To run, `LD_LIBRARY_PATH` must include compiled library (for example by sourcing the provided env.sh) 

The ``real software'' to run the station: 
  See nuphase-ice-software 



