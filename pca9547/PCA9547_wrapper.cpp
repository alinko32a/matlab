
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
#ifndef MATLAB_MEX_FILE
#include <Arduino.h>
#include <math.h>

#include <Wire.h>
#include <Wire.cpp>
#include <utility/twi.h>
#include <utility/twi.c>
#include "chgI2CMultiplexer.cpp"

#include "Adafruit_INA219.h"
#include "Adafruit_INA219.cpp"

Adafruit_INA219 sensor219_1;
Adafruit_INA219 sensor219_2;
#endif
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
 
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern "C" void PCA9547_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
// Ｉ２Ｃの初期化マスターとする
Wire.begin() ;

ChgI2CMultiplexer(0x77,1);
sensor219_1.begin();

ChgI2CMultiplexer(0x77,2);
sensor219_2.begin();
#endif
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern "C" void PCA9547_Outputs_wrapper(real_T *y0)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef MATLAB_MEX_FILE
// I2Cマルチプレクサー(PCA9547)を1チャンネルに切り換える
ChgI2CMultiplexer(0x77,1);
y0[0] = sensor219_1.getCurrent_mA();
        
ChgI2CMultiplexer(0x77,2);
y0[1] = sensor219_2.getCurrent_mA();

#endif
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


