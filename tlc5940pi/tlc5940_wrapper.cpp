
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

#include "tlc-controller.h"
#include "raspberry-gpio.h"

#include <wiringPi.h>

# endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1

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
extern "C" void tlc5940_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

wiringPiSetup();

RaspberryGPIOPin tlc_sin(1);
RaspberryGPIOPin tlc_sclk(14);
RaspberryGPIOPin tlc_blank(4);
RaspberryGPIOPin tlc_dcprg(5);
RaspberryGPIOPin tlc_vprg(6); // Not used in this example
RaspberryGPIOPin tlc_xlat(10);
RaspberryGPIOPin tlc_gsclk(11);

tlc_sin.setOutput();
tlc_sclk.setOutput();
tlc_blank.setOutput();
tlc_dcprg.setOutput();
tlc_vprg.setOutput();
tlc_xlat.setOutput();
tlc_gsclk.setOutput();

SingleTLCController tlc_controller(&tlc_sin, &tlc_sclk, &tlc_blank, &tlc_dcprg, &tlc_vprg, &tlc_xlat, &tlc_gsclk);
    
# endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void tlc5940_Outputs_wrapper(const real_T *u0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
# ifndef MATLAB_MEX_FILE

tlc_controller.setChannel(1, 1000);
tlc_controller.update();

# endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


