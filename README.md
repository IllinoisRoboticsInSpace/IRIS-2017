# IRIS-2017
Code in development for IRIS 2017 autonomous robot

Building
--------

1. Install dependencies  
  
  <code>sudo apt-get install libfreenect-dev libopencv-dev &nbsp;&nbsp;<i> &laquo;and other libraries&raquo; </i></code>  
  
2. Go to the project build directories:

  <code> cd algorithms/<i>test_...</i> </code> or <code> cd algorithms/<i>release_...</i> </code>

3. Make the build directory (this directory is never commited or uploaded)

  `mkdir build`

  `cd build`

4. Create the cmake building structures

  `cmake ..`

5. Compile and link using make

  `make`

6. Execute the resulting program. Don'f forget to plug in the hardware!

  <code>./<i>&laquo;program name&raquo;</i></code>

