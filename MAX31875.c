/** \file MAX31875.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: MAX31875.c
 *         Description: Handling of the MAX31875 temperature sensor
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

#include "max32660.h"
#include "i2c.h"
#include <stdlib.h>

#include "HomebusSlave.h"


/***************************************************************//**
   \fn InitMAX31875()
   \brief MAX31875 initialization

   Initialize the MAX31875 for 8 bit resolution.
********************************************************************/
void InitMAX31875(void)
{
  uint8_t writeData[2];

  writeData[0]=1;  //Register Address: Configuration Register
  writeData[1]=6;  //Configuration: 8 Bit Resolution
  I2C_MasterWrite(MXC_I2C1, 0x90, writeData, 2, 0);
}


/***************************************************************//**
   \fn GetTemperature()
   \return Temperature (°C)
   \brief Read temperature sensor

   Read the temperature sensor and return the temperatur in °C.
********************************************************************/
int GetTemperature(void)
{
  uint8_t writeData[1];
  uint8_t readData[2];
  int16_t temperature;

  writeData[0]=0;  //Register Address: Temperature Register
  I2C_MasterWrite(MXC_I2C1, 0x90, writeData, 1, 1);
  I2C_MasterRead(MXC_I2C1, 0x90, readData, 2, 0);

  temperature=(readData[0] << 8)|readData[1];
  temperature>>=4;

  return (int) temperature/16;
}
