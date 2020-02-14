
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

#include <fcntl.h> // open
#include <inttypes.h> // uint8_t, etc
#include <linux/i2c-dev.h> // I2C bus definitions
#include "read_buf.c"

int I2CFile;

# endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void ads1015_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

I2CFile = open("/dev/i2c-1", O_RDWR);		// Open the I2C device

# endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void ads1015_Outputs_wrapper(real_T *y0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

ioctl(I2CFile, I2C_SLAVE, 0x48);   // Specify the address of the I2C Slave to communicate with
y0[0] = read_buf(0xC3);
y0[1] = read_buf(0xD3);
y0[2] = read_buf(0xE3);
y0[3] = read_buf(0xF3);

ioctl(I2CFile, I2C_SLAVE, 0x49);   // Specify the address of the I2C Slave to communicate with
y0[4] = read_buf(0xC3);
y0[5] = read_buf(0xD3);

# endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Terminate function
 *
 */
void ads1015_Terminate_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

close(I2CFile);

# endif
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

