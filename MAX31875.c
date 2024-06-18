/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

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
