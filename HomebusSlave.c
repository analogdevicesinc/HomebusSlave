/** \file HomebusSlave.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: HomebusSlave.c
 *         Description: Main program for the Homebus slave
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#include <stdlib.h>
#include "max32660.h"
#include "gpio.h"
#include "spimss.h"
#include "i2c.h"
#include "HomebusSlave.h"
#include "Globals.h"
#include "SysTick.h"
#include "TMC5130.h"
#include "MAX31875.h"
#include "RefSearch.h"
#include "TMCL.h"

const char VersionString[]="0026V100";  //<! Version information for the TMCL-IDE
gpio_cfg_t led_out;               //<! Output for LED
gpio_cfg_t enable_out;            //<! Output for TMC5130 ENABLE pin
gpio_cfg_t diag0_in;              //<! Input for TMC5130 DIAG0 pin
gpio_cfg_t diag1_in;              //<! Input for TMC5130 DIAG1 pin
sys_cfg_spimss_t SPICfg={MAP_A};  //<! SPI MSS interface mappinmg
uint32_t Delay;                   //<! used for delays
uint8_t ActualAxis;               //<! Actual axis being processed
uint8_t StopOnStallState[N_O_MOTORS];


/***************************************************************//**
   \fn InitIO()
   \brief I/O initialization

   Initialize all used I/O ports of the MAX32660.
********************************************************************/
void InitIO(void)
{
  enable_out.port = PORT_0;
  enable_out.mask = PIN_7;
  enable_out.pad = GPIO_PAD_NONE;
  enable_out.func = GPIO_FUNC_OUT;
  GPIO_Config(&enable_out);

  diag0_in.port = PORT_0;
  diag0_in.mask = PIN_8;
  diag0_in.pad = GPIO_PAD_PULL_UP;
  diag0_in.func = GPIO_FUNC_IN;
  GPIO_Config(&diag0_in);

  diag1_in.port = PORT_0;
  diag1_in.mask = PIN_9;
  diag1_in.pad = GPIO_PAD_PULL_UP;
  diag1_in.func = GPIO_FUNC_IN;
  GPIO_Config(&diag1_in);
}


/***************************************************************//**
   \fn InitSPI()
   \brief SPI initialization

   Initialize the SPI MSS interface for use with the TMC5130.
********************************************************************/
void InitSPI(void)
{
  SPIMSS_Init(MXC_SPIMSS, 3, 1000000, &SPICfg);
}


/***************************************************************//**
   \fn InitI2C()
   \brief I2C initialization

   Initialize I2C interface for use with the MAX31875.
********************************************************************/
void InitI2C(void)
{
  I2C_Init(MXC_I2C1, I2C_FASTPLUS_MODE, NULL);
}


/***************************************************************//**
   \fn ProcessStallGuard()
   \brief StallGuard functionality

   Do what is necessary for the StallGuard functionality. Must be
   called regularly.
********************************************************************/
void ProcessStallGuard(void)
{
  uint32_t RampStat;

  RampStat=ReadTMC5130Int(ActualAxis, TMC5130_RAMPSTAT);

  //Final stop after stall event
  if(RampStat & TMC5130_RS_EV_STOP_SG)
  {
    HardStop(ActualAxis);
    WriteTMC5130Int(WHICH_5130(ActualAxis), TMC5130_SWMODE, ReadTMC5130Int(WHICH_5130(ActualAxis), TMC5130_SWMODE) & ~TMC5130_SW_SG_STOP);
    StallFlag[ActualAxis]=TRUE;
  }

  //Switch StallGuard on and off depending on the actual velocity
  if(StallVMin[ActualAxis]>0 && abs(ReadTMC5130Int(ActualAxis, TMC5130_VACTUAL))>StallVMin[ActualAxis])
  {
    if(!StopOnStallState[ActualAxis])
    {
      WriteTMC5130Int(ActualAxis, TMC5130_SWMODE, ReadTMC5130Int(ActualAxis, TMC5130_SWMODE) | TMC5130_SW_SG_STOP);
      StopOnStallState[ActualAxis]=TRUE;
    }
  }
  else
  {
    if(StopOnStallState[ActualAxis])
    {
      if(ReadTMC5130Int(WHICH_5130(ActualAxis), TMC5130_VMAX)==0 || abs(ReadTMC5130Int(WHICH_5130(ActualAxis), TMC5130_VACTUAL))>0)
      {
        WriteTMC5130Int(ActualAxis, TMC5130_SWMODE, ReadTMC5130Int(ActualAxis, TMC5130_SWMODE) & ~TMC5130_SW_SG_STOP);
      }
      StopOnStallState[ActualAxis]=FALSE;
    }
  }

  //Reset TMC51xx event flags after they have been processed
  if(RampStat & TMC5130_RS_EV_POSREACHED)
    WriteTMC5130Int(WHICH_5130(ActualAxis), TMC5130_RAMPSTAT, TMC5130_RS_EV_POSREACHED);
  if(RampStat & TMC5130_RS_EV_STOP_SG)
    WriteTMC5130Int(WHICH_5130(ActualAxis), TMC5130_RAMPSTAT, TMC5130_RS_EV_STOP_SG);

  ProcessRefSearch(ActualAxis);

  //Next axis
  ActualAxis++;
  if(ActualAxis>=N_O_MOTORS) ActualAxis=0;
}


//Main program
void main()
{
  InitSysTick();
  InitIO();
  InitSPI();
  InitMotorDrivers();
  InitI2C();
  InitMAX31875();
  HomebusInit(230400);
  InitTMCL();

  GPIO_OutClr(&enable_out);

  Delay=GetSysTimer();
  for(;;)
  {
    if(abs(GetSysTimer()-Delay)>1000)
    {
      Delay=GetSysTimer();
    }

    ProcessCommand();
    ProcessStallGuard();
  }
}
