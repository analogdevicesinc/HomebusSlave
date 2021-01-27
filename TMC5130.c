/** \file TMC5130.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: TMC5130.c
 *         Description: TMC5130 interface functions
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
#include <math.h>
#include "max32660.h"
#include "spi.h"
#include "spimss.h"
#include "bits.h"
#include "math.h"

#include "HomebusSlave.h"
#include "TMC5130.h"
#include "SysTick.h"
#include "Globals.h"

#define VEL_FACTOR 0.7451                 //fClk/2 / 2^23   (Internal clock, typical fClk=12.5MHz)
#define ACC_FACTOR 71.054274              //fClk^2 / (512*256) / 2^24   (Internal clock, typical fClk=12.5MHz)

//This table shows which TMC5130 register can be read back (0=no, 1=yes).
static const uint8_t TMC5130RegisterReadable[128]={
1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //00..0f
0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //10..1f
1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,    //20..2f
0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0,    //30..3f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //40..4f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //50..5f
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1,    //60..6f
0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    //70..7f
};

static int TMC5130SoftwareCopy[128][N_O_MOTORS];    //!< Software copy of all registers
static uint8_t DriverDisableFlag[N_O_MOTORS];       //!< Flags used for switching off a motor driver via TOff
static uint8_t LastTOffSetting[N_O_MOTORS];         //!< Last TOff setting before switching off the driver


/***************************************************************//**
   \fn WriteTMC5130Datagram(uint8_t Which5130, uint8_t Address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
   \brief Write bytes to a TMC5130 register
   \param Which5130  Index of TMC5130 to be used (with stepRocker always 0)
   \param Address    Register adress (0x00..0x7f)
   \param x1        First byte to write (MSB)
   \param x2        Second byte to write
   \param x3        Third byte to write
   \param x4        Fourth byte to write (LSB)

  This is a low level function for writing data to a TMC5130 register
  (32 bit value split up into four bytes).
********************************************************************/
void WriteTMC5130Datagram(uint8_t Which5130, uint8_t Address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
  int Value;
  spimss_req_t SPIRequest;
  uint8_t SPITxData[5];
  uint8_t SPIRxData[5];

  //Emulate W1C-Bits of the TMC5160
  if((Address & 0x7f) == TMC5130_RAMPSTAT)
  {
    Value=x1;
    Value<<=8;
    Value|=x2;
    Value<<=8;
    Value|=x3;
    Value<<=8;
    Value|=x4;
    Value&=(BIT12|BIT7|BIT6|BIT3|BIT2);
    TMC5130SoftwareCopy[Address & 0x7f][Which5130]&= ~Value;
    return;
  }

  //Write to TMC5130 register
  SPITxData[0]=Address|0x80;
  SPITxData[1]=x1;
  SPITxData[2]=x2;
  SPITxData[3]=x3;
  SPITxData[4]=x4;
  SPIRequest.ssel=0;
  SPIRequest.deass=1;
  SPIRequest.tx_data=SPITxData;
  SPIRequest.rx_data=SPIRxData;
  SPIRequest.len=5;
  SPIRequest.bits=8;
  SPIRequest.callback=NULL;
  SPIMSS_MasterTrans(MXC_SPIMSS, &SPIRequest);

  //Update software copy
  Value=x1;
  Value<<=8;
  Value|=x2;
  Value<<=8;
  Value|=x3;
  Value<<=8;
  Value|=x4;
  TMC5130SoftwareCopy[Address & 0x7f][Which5130]=Value;
}


/***************************************************************//**
   \fn WriteTMC5130Int(uint8_t Which5130, uint8_t Address, int Value)
   \brief Write a 32 bit value to a TMC5130 register
   \param Which5130  Index of TMC5130 to be used (with stepRocker always 0)
   \param Address    Registeradresse (0x00..0x7f)
   \param Value      Value to be written

  This is a low level function for writing data to a TMC5130 register
  (32 bit value).
********************************************************************/
void WriteTMC5130Int(uint8_t Which5130, uint8_t Address, int Value)
{
  spimss_req_t SPIRequest;
  uint8_t SPITxData[5];
  uint8_t SPIRxData[5];

  if(Which5130!=0) return;

  //Emulate W1C-Bits emulieren of the TMC5130
  if((Address & 0x7f) == TMC5130_RAMPSTAT)
  {
    Value&=(BIT12|BIT7|BIT6|BIT3|BIT2);
    TMC5130SoftwareCopy[Address & 0x7f][Which5130]&= ~Value;
    return;
  }

  //Write to TMC5130 register
  SPITxData[0]=Address|0x80;
  SPITxData[1]=Value >> 24;
  SPITxData[2]=Value >> 16;
  SPITxData[3]=Value >> 8;
  SPITxData[4]=Value & 0xff;
  SPIRequest.ssel=0;
  SPIRequest.deass=1;
  SPIRequest.tx_data=SPITxData;
  SPIRequest.rx_data=SPIRxData;
  SPIRequest.len=5;
  SPIRequest.bits=8;
  SPIRequest.callback=NULL;
  SPIMSS_MasterTrans(MXC_SPIMSS, &SPIRequest);

  //Update software copy
  TMC5130SoftwareCopy[Address & 0x7f][Which5130]=Value;
}


/***************************************************************//**
   \fn ReadTMC5130Int(uint8_t Which5130, uint8_t Address)
   \brief Write a 32 bit value to a TMC5130 register
   \param Which5130  Index of TMC5130 to be used (with stepRocker always 0)
   \param Address    Registeradresse (0x00..0x7f)
   \return           Value read from the register

  This is a low level function for writing data to a TMC5130 register
  (32 bit value).
********************************************************************/
int ReadTMC5130Int(uint8_t Which5130, uint8_t Address)
{
  int Value;
  spimss_req_t SPIRequest;
  uint8_t SPITxData[5];
  uint8_t SPIRxData[5];

  if(Which5130!=0) return 0;

  Address&=0x7f;
  if(TMC5130RegisterReadable[Address])
  {
    //Register readavle => read from TMC5130.
    //Two read accesses are needed for this.
    SPITxData[0]=Address;
    SPITxData[1]=0;
    SPITxData[2]=0;
    SPITxData[3]=0;
    SPITxData[4]=0;
    SPIRequest.ssel=0;
    SPIRequest.deass=1;
    SPIRequest.tx_data=SPITxData;
    SPIRequest.rx_data=SPIRxData;
    SPIRequest.len=5;
    SPIRequest.bits=8;
    SPIRequest.callback=NULL;
    SPIMSS_MasterTrans(MXC_SPIMSS, &SPIRequest);

    //Always use register 0 (GCONF) for the second access.
    SPITxData[0]=0;
    SPIMSS_MasterTrans(MXC_SPIMSS, &SPIRequest);
    Value=(SPIRxData[1]<<24)|(SPIRxData[2]<<16)|(SPIRxData[3]<<8)|SPIRxData[4];

    //Emulate W1C-Bits of the TMC5160
    #if !defined(DEVTYPE_TMC5160)
    if(Address==TMC5130_RAMPSTAT)
    {
      TMC5130SoftwareCopy[Address][Which5130]&=(BIT12|BIT7|BIT6|BIT3|BIT2);
      TMC5130SoftwareCopy[Address][Which5130]|=Value;
      Value=TMC5130SoftwareCopy[Address][Which5130];
    }
    #endif

    //Sign extend VACTUAL register (24 bits => 32 bits)
    if(Address==0x22 || Address==0x42)
    {
      if(Value & BIT23) Value|=0xff000000;
    }

    return Value;
  }
  else
  {
    //Register not readable => return software copy
    return TMC5130SoftwareCopy[Address][Which5130];
  }
}


/***************************************************************//**
   \fn SetTMC5130ChopperTOff(uint8_t Motor, uint8_t TOff)
   \brief Set the TOff parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param TOff    TOff parameter

  This function sets the TOff parameter.
********************************************************************/
void SetTMC5130ChopperTOff(uint8_t Motor, uint8_t TOff)
{
  uint32_t Value;

  if(!DriverDisableFlag[Motor])
  {
    Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xfffffff0;
    WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | (TOff & 0x0f));
  }
  LastTOffSetting[Motor]=TOff;
}

/***************************************************************//**
   \fn SetTMC5130ChopperHysteresisStart(uint8_t Motor, uint8_t HysteresisStart)
   \brief Set the HSTART parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param HysteresisStart    Hysteresis start parameter.

  This function sets the HSTART parameter.
********************************************************************/
void SetTMC5130ChopperHysteresisStart(uint8_t Motor, uint8_t HysteresisStart)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xffffff8f;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((HysteresisStart & 0x07) << 4));
}

/***************************************************************//**
   \fn SetTMC5130ChopperHysteresisEnd(uint8_t Motor, uint8_t HysteresisEnd)
   \brief Set the HEND parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param HysteresisEnd    Hysteresis end parameter.

  This function sets the HEND parameter.
********************************************************************/
void SetTMC5130ChopperHysteresisEnd(uint8_t Motor, uint8_t HysteresisEnd)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xfffff87f;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((HysteresisEnd & 0x0f) << 7));
}

/***************************************************************//**
   \fn SetTMC5130ChopperBlankTime(uint8_t Motor, uint8_t BlankTime)
   \brief Set the chopper blank time parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param BlankTime    Chopper blank time.

  This function sets the chopper blank time parameter.
********************************************************************/
void SetTMC5130ChopperBlankTime(uint8_t Motor, uint8_t BlankTime)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xfffe7fff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((BlankTime & 0x03) << 15));
}

/***************************************************************//**
   \fn SetTMC5130ChopperSync(uint8_t Motor, uint8_t Sync)
   \brief Set the chopper synchronization parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \param Sync    Chopper sync time.

  This function sets the chopper synchronization parameter.
********************************************************************/
void SetTMC5130ChopperSync(uint8_t Motor, uint8_t Sync)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xff0fffff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((Sync & 0x0f) << 20));
}

/***************************************************************//**
   \fn SetTMC5130ChopperMStepRes(uint8_t Motor, uint8_t MRes)
   \brief Set microstep resolution.
   \param Motor   Axis number (with stepRocker always 0)
   \param MRes    Microstep resolution (0..7).

  This function sets the microstep resolution.
********************************************************************/
void SetTMC5130ChopperMStepRes(uint8_t Motor, uint8_t MRes)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xf0ffffff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((MRes & 0x0f) << 24));
}

/***************************************************************//**
   \fn SetTMC5130ChopperDisableShortToGround(uint8_t Motor, uint8_t Disable)
   \brief Disable the short to ground detection.
   \param Motor   Axis number (with stepRocker always 0)
   \param Disable   TRUE: short to ground detection off.\n
                    FALSE: short to ground detection on.

  This function disables or enables the short to ground detection.
********************************************************************/
void SetTMC5130ChopperDisableShortToGround(uint8_t Motor, uint8_t Disable)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(Disable)
    Value|=BIT30;
  else
    Value&= ~BIT30;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperVHighChm(uint8_t Motor, uint8_t VHighChm)
   \brief Switch off/on VHIGHCHM flag.
   \param Motor   Axis number (with stepRocker always 0)
   \param VHighChm  TRUE/FALSE.

  This function disables or enables the VHIGHCHM flag.
********************************************************************/
void SetTMC5130ChopperVHighChm(uint8_t Motor, uint8_t VHighChm)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(VHighChm)
    Value|=BIT19;
  else
    Value&= ~BIT19;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperVHighFs(uint8_t Motor, uint8_t VHighFs)
   \brief Switch off/on VHIGHFS flag.
   \param Motor   Axis number (with stepRocker always 0)
   \param VHighFs   TRUE/FALSE.

  This function disables or enables the VHIGHFS flag.
********************************************************************/
void SetTMC5130ChopperVHighFs(uint8_t Motor, uint8_t VHighFs)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(VHighFs)
    Value|=BIT18;
  else
    Value&= ~BIT18;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperConstantTOffMode(uint8_t Motor, uint8_t ConstantTOff)
   \brief Switch off/on constant chopper mode.
   \param Motor   Axis number (with stepRocker always 0)
   \param ConstantTOff  Constant TOff mode switch: TRUE/FALSE.

  This function disables or enables the constant chopper mode.
********************************************************************/
void SetTMC5130ChopperConstantTOffMode(uint8_t Motor, uint8_t ConstantTOff)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(ConstantTOff)
    Value|=BIT14;
  else
    Value&= ~BIT14;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperRandomTOff(uint8_t Motor, uint8_t RandomTOff)
   \brief Switch off/on random TOff mode.
   \param Motor   Axis number (with stepRocker always 0)
   \param RandomTOff  TRUE/FALSE.

  This function disables or enables the random chopper mode.
********************************************************************/
void SetTMC5130ChopperRandomTOff(uint8_t Motor, uint8_t RandomTOff)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(RandomTOff)
    Value|=BIT13;
  else
    Value&= ~BIT13;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperDisableFastDecayComp(uint8_t Motor, uint8_t Disable)
   \brief Switch off/on FastDecayComp.
   \param Motor   Axis number (with stepRocker always 0)
   \param Disable  TRUE/FALSE.

  This function disables or enables the fast decay comparator.
********************************************************************/
void SetTMC5130ChopperDisableFastDecayComp(uint8_t Motor, uint8_t Disable)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  if(Disable)
    Value|=BIT12;
  else
    Value&= ~BIT12;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn SetTMC5130ChopperFastDecayTime(uint8_t Motor, uint8_t Time)
   \brief Set the fast decay time.
   \param Motor   Axis number (with stepRocker always 0)
   \param Time    Fast decay time (0..15).

  This function sets the fast decay time.
********************************************************************/
void SetTMC5130ChopperFastDecayTime(uint8_t Motor, uint8_t Time)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xffffff8f;

  if(Time & BIT3)
    Value|=BIT11;
  else
    Value&= ~BIT11;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((Time & 0x07) << 4));
}

/***************************************************************//**
   \fn SetTMC5130ChopperSineWaveOffset(uint8_t Motor, uint8_t Offset)
   \brief Set the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \param Offset  Sine wave offset(0..15).

  This function sets the sine wave offset.
********************************************************************/
void SetTMC5130ChopperSineWaveOffset(uint8_t Motor, uint8_t Offset)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0xfffff87f;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value | ((Offset & 0x0f) << 7));
}

/***************************************************************//**
   \fn SetTMC5130ChopperVSenseMode(uint8_t Motor, uint8_t Offset)
   \brief Set the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \param Offset  Sine wave offset(0..15).

  This function sets the VSense mode.
********************************************************************/
void SetTMC5130ChopperVSenseMode(uint8_t Motor, uint8_t Mode)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);

  if(Mode)
    Value|=BIT17;
  else
    Value&= ~BIT17;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF, Value);
}

/***************************************************************//**
   \fn GetTMC5130ChopperTOff(uint8_t Motor)
   \brief Read the TOff parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TOff parameter

  This function reads the TOff parameter.
********************************************************************/
uint8_t GetTMC5130ChopperTOff(uint8_t Motor)
{
  if(!DriverDisableFlag[Motor])
    return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & 0x0000000f;
  else
    return LastTOffSetting[Motor];
}

/***************************************************************//**
   \fn GetTMC5130ChopperHysteresisStart(uint8_t Motor)
   \brief Read the HSTART parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Hysteresis start parameter.

  This function reads the HSTART parameter.
********************************************************************/
uint8_t GetTMC5130ChopperHysteresisStart(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 4) & 0x07;
}

/***************************************************************//**
   \fn GetTMC5130ChopperHysteresisEnd(uint8_t Motor)
   \brief Read the HEND parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Hysteresis end parameter.

  This function sets the HEND parameter.
********************************************************************/
uint8_t GetTMC5130ChopperHysteresisEnd(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 7) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5130ChopperBlankTime(uint8_t Motor)
   \brief Read the chopper blank time parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Chopper blank time.

  This function reads the chopper blank time parameter.
********************************************************************/
uint8_t GetTMC5130ChopperBlankTime(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 15) & 0x03;
}

/***************************************************************//**
   \fn GetTMC5130ChopperSync(uint8_t Motor)
   \brief Read the chopper synchronization parameter.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Chopper synchronization.

  This function reads the chopper synchronization parameter.
********************************************************************/
uint8_t GetTMC5130ChopperSync(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 20) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5130ChopperMStepRes(uint8_t Motor)
   \brief Read microstep resolution.
   \param Motor   Axis number (with stepRocker always 0)
   \return        Microstep resolution (0..7).

  This function reads the microstep resolution.
********************************************************************/
uint8_t GetTMC5130ChopperMStepRes(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 24) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5130ChopperDisableShortToGround(uint8_t Motor)
   \brief Check if short to ground detection is disabled.
   \param Motor   Axis number (with stepRocker always 0)
   \return      TRUE: short to ground detection off.\n
                FALSE: short to ground detection on.

  This function checks if the short to ground detection is enabled or
  disabled.
********************************************************************/
uint8_t GetTMC5130ChopperDisableShortToGround(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT30 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperVHighChm(uint8_t Motor)
   \brief Check VHIGHCHM flag.
   \param Motor   Axis number (with stepRocker always 0)
   \return   State of VHighChm bit.

  This function reads back the VHIGHCHM flag.
********************************************************************/
uint8_t GetTMC5130ChopperVHighChm(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT19 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperVHighFs(uint8_t Motor)
   \brief Check VHIGHFS flag.
   \param Motor   Axis number (with stepRocker always 0)
   \return   State of VHighFs bit.

  This function reads back the VHIGHFS flag.
********************************************************************/
uint8_t GetTMC5130ChopperVHighFs(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT18 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperConstantTOffMode(uint8_t Motor)
   \brief Check if constant chopper mode is switched on or off.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TRUE/FALSE

  This function checks if the constant chopper mode is selected.
********************************************************************/
uint8_t GetTMC5130ChopperConstantTOffMode(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT14 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperRandomTOff(uint8_t Motor)
   \brief Check if random TOff mode is switched on.
   \param Motor   Axis number (with stepRocker always 0)
   \return        TRUE/FALSE.

  This function disables or enables the random chopper mode.
********************************************************************/
uint8_t GetTMC5130ChopperRandomTOff(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT13 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperDisableFastDecayComp(uint8_t Motor)
   \brief Read back the fast decay comparator setting.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Fast decay comparator setting.

  This function reads back the fast decay comparator disable setting.
********************************************************************/
uint8_t GetTMC5130ChopperDisableFastDecayComp(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT12 ? 1:0;
}

/***************************************************************//**
   \fn GetTMC5130ChopperFastDecayTime(uint8_t Motor)
   \brief Read back the fast decay time.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Fast decay time (0..15).

  This function reads back the fast decay time.
********************************************************************/
uint8_t GetTMC5130ChopperFastDecayTime(uint8_t Motor)
{
  uint32_t Value;
  uint8_t Time;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF);
  Time=(Value >> 4) & 0x07;
  if(Value & BIT11) Time|=BIT3;

  return Time;
}

/***************************************************************//**
   \fn GetTMC5130ChopperSineWaveOffset(uint8_t Motor)
   \brief Read the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \return   Sine Wave Offset (0..15).

  This function reads back the sine wave offset.
********************************************************************/
uint8_t GetTMC5130ChopperSineWaveOffset(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) >> 7) & 0x0f;
}

/***************************************************************//**
   \fn GetTMC5130ChopperSineWaveOffset(uint8_t Motor)
   \brief Read the sine offset value.
   \param Motor   Axis number (with stepRocker always 0)
   \return   VSense mode (0 or 1).

  This function reads back the VSense mode.
********************************************************************/
uint8_t GetTMC5130ChopperVSenseMode(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_CHOPCONF) & BIT17 ? 1:0;
}


/*************************************************************************//**
  \fn SetTMC5130SmartEnergyUpStep(uint8_t Motor, uint8_t UpStep)
  \brief Set smart energy up step
  \param Motor        Axis number (with stepRocker always 0)
  \param UpStep       up step width (0..3)

  This function sets the current up step width used with coolStep, where 0 ist
  the lowest and 3 is the highest up step width.
*****************************************************************************/
void SetTMC5130SmartEnergyUpStep(uint8_t Motor, uint8_t UpStep)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0xffffff9f;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value | ((UpStep & 0x03) << 5));
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyDownStep(uint8_t Motor, uint8_t DownStep)
  \brief Set smart energy down step
  \param Motor          Axis number (with stepRocker always 0)
  \param DownStep       down step speed (0..3)

  This function sets the current down step speed used with coolStep, where 0 ist
  the highest and 3 is the lowest speed.
*****************************************************************************/
void SetTMC5130SmartEnergyDownStep(uint8_t Motor, uint8_t DownStep)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0xffff9fff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value | ((DownStep & 0x03) << 13));
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyStallLevelMax(uint8_t Motor, uint8_t Max)
  \brief Set smart enery hysteresis width
  \param Motor          Axis number (with stepRocker always 0)
  \param Max            hysteresis width (0..15)

  This function sets the SEMAX parameter which defines the width of the
  smart energy stall level hysteresis.
*****************************************************************************/
void SetTMC5130SmartEnergyStallLevelMax(uint8_t Motor, uint8_t Max)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0xfffff0ff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value | ((Max & 0x0f) << 8));
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyStallLevelMin(uint8_t Motor, uint8_t Min)
  \brief Set smart energy hysteresis start
  \param Motor          Axis number (with stepRocker always 0)
  \param Min            minimum stall level (0..15)

  This function sets the start point of the hysteresis used for coolStep.
  A value of 0 completely turns off coolStep.
*****************************************************************************/
void SetTMC5130SmartEnergyStallLevelMin(uint8_t Motor, uint8_t Min)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0xfffffff0;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value | (Min & 0x0f));
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyStallThreshold(uint8_t Motor, char Threshold)
  \brief Set stallGuard threshold value
  \param Motor       Axis number (with stepRocker always 0)
  \param Threshold   stallGuard threshold (-63..+63)

  This function sets the stallGuard threshold value.
*****************************************************************************/
void SetTMC5130SmartEnergyStallThreshold(uint8_t Motor, char Threshold)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0xff00ffff;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value | ((Threshold & 0xff) << 16));
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyIMin(uint8_t Motor, uint8_t IMin)
  \brief Set smart energy minimum current
  \param Motor      Axis number (with stepRocker always 0)
  \param IMin       Minimum current (0=1/2, 1=1/4 of current setting)

  This function sets the minimum current used with coolStep, which can
  be either 1/2 or 1/4 of the normal current setting.
*****************************************************************************/
void SetTMC5130SmartEnergyIMin(uint8_t Motor, uint8_t IMin)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF);
  if(IMin)
    Value|=BIT15;
  else
    Value&= ~BIT15;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130SmartEnergyFilter(uint8_t Motor, uint8_t Filter)
  \brief Set stallGuard filter
  \param Motor             Axis number (with stepRocker always 0)
  \param Filter            stallGuard filter (0=off, 1=on)

  This function turns the stallGuard filter on or off.
*****************************************************************************/
void SetTMC5130SmartEnergyFilter(uint8_t Motor, uint8_t Filter)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF);
  if(Filter)
    Value|=BIT24;
  else
    Value&= ~BIT24;

  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF, Value);
}


/*************************************************************************//**
  \fn GetTMC5130SmartEnergyUpStep(uint8_t Motor)
  \brief Get current up step width
  \param Motor  Axis number (with stepRocker always 0)
  \return SEUP value

  This function reads back the current up step setting.
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyUpStep(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) >> 5) & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyDownStep(uint8_t Motor)
  \brief Get current down step speed
  \param Motor  Axis number (with stepRocker always 0)
  \return Current down step speed

  This function reads back the smart energy current down step speed setting.
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyDownStep(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) >> 13) & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyStallLevelMax(uint8_t Motor)
  \brief Get hystersis width
  \param Motor  Axis number (with stepRocker always 0)
  \return SEMAX value

  This function reads back the stall level maximum value (which is the coolStep
  hysteresis width).
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyStallLevelMax(uint8_t Motor)
{
  return (ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) >> 8) & 0x0f;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyStallLevelMin(uint8_t Motor)
  \brief Get hysteresis start
  \param Motor  Axis number (with stepRocker always 0)
  \return hysteresis start

  This function reads back the smart energy minimum stall level (which is the
  start of the coolStep hystetesis).
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyStallLevelMin(uint8_t Motor)
{
  return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & 0x0f;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyStallThreshold(uint8_t Motor)
  \brief Get stallGuard threshold setting
  \param Motor  Axis number (with stepRocker always 0)
  \return stallGuard threshold value

  This function reads back the stallGuard threshold value.
*****************************************************************************/
int GetTMC5130SmartEnergyStallThreshold(uint8_t Motor)
{
  int Value;

  Value=(ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) >> 16) & 0xff;
  if(Value & BIT7) Value|=0xffffff00;

  return Value;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyIMin(uint8_t Motor)
  \brief Get minimum current
  \param Motor  Axis number (with stepRocker always 0)
  \return Minimum currren

  This function reads back the smart energy minimum current setting.
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyIMin(uint8_t Motor)
{
  if(ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & BIT15)
    return 1;
  else
    return 0;
}

/*************************************************************************//**
  \fn GetTMC5130SmartEnergyFilter(uint8_t Motor)
  \brief Get stallGuard filter
  \param Motor  Axis number (with stepRocker always 0)
  \return stallGuard filter (0=off, 1=on)

  This function reads back the stallGuard filter setting.
*****************************************************************************/
uint8_t GetTMC5130SmartEnergyFilter(uint8_t Motor)
{
  if(ReadTMC5130Int(WHICH_5130(Motor), TMC5130_COOLCONF) & BIT24)
    return 1;
  else
    return 0;
}

/*************************************************************************//**
  \fn SetTMC5130PWMFreewheelMode(uint8_t Motor, uint8_t Mode)
  \brief Set freewheeling mode
  \param Motor  Axis number (with stepRocker always 0)
  \param Mode  Freeheeling mode (0=off, 1=LS short, 2=HS short, 3=freewheel)

  This function selects the freewheeling mode. StealthChop has to be active
  to make this setting become active.
*****************************************************************************/
void SetTMC5130PWMFreewheelMode(uint8_t Motor, uint8_t Mode)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & 0xffcfffff;
  Mode&=0x03;
  Value|=Mode << 20;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130PWMSymmetric(uint8_t Motor, uint8_t Symmetric)
  \brief Set symmetric PWM mode
  \param Motor  Axis number (with stepRocker always 0)
  \param Symmetric  TRUE or FALSE

  This function switches PWM symmetric mode on or off.
*****************************************************************************/
void SetTMC5130PWMSymmetric(uint8_t Motor, uint8_t Symmetric)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF);
  if(Symmetric)
    Value|=BIT19;
  else
    Value&= ~BIT19;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130PWMAutoscale(uint8_t Motor, uint8_t Autoscale)
  \brief Set autoscale mode for stealthChop
  \param Motor       Axis number (with stepRocker always 0)
  \param Autoscale   1=on,  0=off

  This function switches on or off the autoscale option for stealthChop.
*****************************************************************************/
void SetTMC5130PWMAutoscale(uint8_t Motor, uint8_t Autoscale)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF);
  if(Autoscale)
    Value|=BIT18;
  else
    Value&= ~BIT18;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130PWMFrequency(uint8_t Motor, uint8_t Frequency)
  \brief Set PWM frequency
  \param Motor  Axis number (with stepRocker always 0)
  \param Frequency  PWM frequency setting (0..3)

  This function sets the TMC5130 PWM frequency.
*****************************************************************************/
void SetTMC5130PWMFrequency(uint8_t Motor, uint8_t Frequency)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & 0xfffcffff;
  Frequency&=0x03;
  Value|=Frequency << 16;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130PWMGrad(uint8_t Motor, uint8_t PWMGrad)
  \brief Set stealthChop PWM gradient
  \param Motor  Axis number (with stepRocker always 0)
  \param PWMGrad  PWM gradient (0..15)

  This function sets the PWM gradient used for stealthChop.
*****************************************************************************/
void SetTMC5130PWMGrad(uint8_t Motor, uint8_t PWMGrad)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & 0xffff00ff;
  Value|=PWMGrad << 8;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn SetTMC5130PWMAmpl(uint8_t Motor, uint8_t PWMAmpl)
  \brief Set stealthChop PWM amplitude
  \param Motor  Axis number (with stepRocker always 0)
  \param PWMAmpl   PWM amplitude (0..255)

  This function sets the PWM amplitude used for stealthChop.
*****************************************************************************/
void SetTMC5130PWMAmpl(uint8_t Motor, uint8_t PWMAmpl)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & 0xffffff00;
  Value|=PWMAmpl;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF, Value);
}

/*************************************************************************//**
  \fn GetTMC5130PWMFreewheelMode(uint8_t Motor)
  \brief Read back freewheeling mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       Freeheeling mode (0=off, 1=LS short, 2=HS short, 3=freewheel)

  This function reads the selected freewheeling mode.
*****************************************************************************/
uint8_t GetTMC5130PWMFreewheelMode(uint8_t Motor)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF);
  Value>>=20;

  return Value & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5130PWMSymmetric(uint8_t Motor)
  \brief Read back symmetric PWM mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       TRUE or FALSE

  This function reads back the PWM symmetric mode.
*****************************************************************************/
uint8_t GetTMC5130PWMSymmetric(uint8_t Motor)
{
  if(ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & BIT19)
    return TRUE;
  else
    return FALSE;
}

/*************************************************************************//**
  \fn GetTMC5130PWMAutoscale(uint8_t Motor)
  \brief Read back stealthChop autoscale mode
  \param Motor  Axis number (with stepRocker always 0)
  \return       1=on\n
                0=off

  This function reads back the state of the autoscale option for stealthChop.
*****************************************************************************/
uint8_t GetTMC5130PWMAutoscale(uint8_t Motor)
{
  if(ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & BIT18)
    return TRUE;
  else
    return FALSE;
}

/*************************************************************************//**
  \fn GetTMC5130PWMFrequency(uint8_t Motor)
  \brief Read back PWM frequency
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM frequency setting (0..3)

  This function reads back the TMC5130 PWM frequency.
*****************************************************************************/
uint8_t GetTMC5130PWMFrequency(uint8_t Motor)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF);
  Value>>=16;

  return Value & 0x03;
}

/*************************************************************************//**
  \fn GetTMC5130PWMGrad(uint8_t Motor)
  \brief Read back stealthChop PWM gradient
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM gradient (0..15)

  This function reads back the PWM gradient used for stealthChop.
*****************************************************************************/
uint8_t GetTMC5130PWMGrad(uint8_t Motor)
{
  uint32_t Value;

  Value=ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF);
  Value>>=8;

  return Value & 0xff;
}

/*************************************************************************//**
  \fn GetTMC5130PWMAmpl(uint8_t Motor)
  \brief Set stealthChop PWM amplitude
  \param Motor  Axis number (with stepRocker always 0)
  \return       PWM amplitude (0..255)

  This function reads back the PWM amplitude used for stealthChop.
*****************************************************************************/
uint8_t GetTMC5130PWMAmpl(uint8_t Motor)
{
   return ReadTMC5130Int(WHICH_5130(Motor), TMC5130_PWMCONF) & 0xff;
}


/***************************************************************//**
   \fn Read5130State(uint8_t Which5130, uint32_t *StallGuard, uint8_t *SmartEnergy, uint8_t *Flags)
   \brief Get TMC5130 status from TMC4361 polling mechanism
   \param Which5130  Index of TMC5130 to be used (with stepRocker always 0)
   \param StallGuard   Pointer at uint32_t for stallGuard value
   \param SmartEnergy  Pointer at uint8_t for smartEnergy value
   \param Flags   Pointer at uint8_t for driver error flags

   This function reads status information from the TMC5130 motor driver.
   The values are extracted from the datagrams.

   NULL pointers can be used for values that are not needed.
********************************************************************/
void Read5130State(uint8_t Which5130, uint32_t *StallGuard, uint8_t *SmartEnergy, uint8_t *Flags)
{
  uint32_t DrvStatus;

  DrvStatus=ReadTMC5130Int(Which5130, TMC5130_DRVSTATUS);
  if(StallGuard!=NULL) *StallGuard=DrvStatus & 0x3ff;
  if(SmartEnergy!=NULL) *SmartEnergy=(DrvStatus>>16) & 0x1f;
  if(Flags!=NULL) *Flags=(DrvStatus>>24) & 0xff;
}


/***************************************************************//**
   \fn InitMotorDrivers(void)
   \brief Initialise all motor drivers

   This function initalizes the software copies of all TMC5130
   registers and sends this basic initialization data to all
   TMC5130 ICs.
********************************************************************/
void InitMotorDrivers(void)
{
  int Delay;
  int i;

  //Short delay (TMC5130 power-on reset)
  Delay=GetSysTimer();
  while(abs(GetSysTimer()-Delay)<10);

  for(i=0; i<N_O_MOTORS; i++)
  {
    WriteTMC5130Int(i, TMC5130_GCONF, 0);
    WriteTMC5130Datagram(i, TMC5130_CHOPCONF, 0x00, 0x01, 0x02, 0x55);
    WriteTMC5130Datagram(i, TMC5130_IHOLD_IRUN, 0x00, 0x07, 0x0f, 0x01);
    WriteTMC5130Int(i, TMC5130_PWMCONF, 0x00050480); //Reset default of PWMCONF
    LastTOffSetting[i]=GetTMC5130ChopperTOff(i);
    DriverDisableFlag[i]=FALSE;

    WriteTMC5130Int(i, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    WriteTMC5130Int(i, TMC5130_XTARGET, 0);
    WriteTMC5130Int(i, TMC5130_XACTUAL, 0);

    WriteTMC5130Int(i, TMC5130_VSTART, 1);
    WriteTMC5130Int(i, TMC5130_A1, 25600/ACC_FACTOR);
    WriteTMC5130Int(i, TMC5130_V1, 25600/VEL_FACTOR);
    WriteTMC5130Int(i, TMC5130_AMAX, 51200/ACC_FACTOR);
    WriteTMC5130Int(i, TMC5130_VMAX, 51200/VEL_FACTOR);
    WriteTMC5130Int(i, TMC5130_DMAX, 51200/ACC_FACTOR);
    WriteTMC5130Int(i, TMC5130_D1, 25600/ACC_FACTOR);
    WriteTMC5130Int(i, TMC5130_VSTOP, 10/VEL_FACTOR);

    WriteTMC5130Int(i, TMC5130_TCOOLTHRS, 1048575);

  }
}

/***************************************************************//**
   \fn DisableTMC5130(uint8_t Motor)
   \brief Disable a motor driver
   \param Motor  Axis number (always 0 on stepRocker)

   Completely switch off a motor driver (by setting its TOff value
   to zero).
********************************************************************/
void DisableTMC5130(uint8_t Motor)
{
  uint8_t TOff;

  if(!DriverDisableFlag[Motor])
  {
    TOff=LastTOffSetting[Motor];
    SetTMC5130ChopperTOff(Motor, 0);
    DriverDisableFlag[Motor]=TRUE;
    LastTOffSetting[Motor]=TOff;
  }
}

/***************************************************************//**
   \fn EnableTMC5130(uint8_t Motor)
   \brief Enable a motor driver
   \param Motor  Axis number (always 0 on stepRocker)

   Re-enable a motor driver (by setting its TOff value back to the
   original value).
********************************************************************/
void EnableTMC5130(uint8_t Motor)
{
  if(DriverDisableFlag[Motor])
  {
    DriverDisableFlag[Motor]=FALSE;
    SetTMC5130ChopperTOff(Motor, LastTOffSetting[Motor]);
  }
}

/***************************************************************//**
   \fn HardStop(uint32_t Motor)
   \brief Stop motor immediately
   \param Motor  Motornummer (always 0 with stealthRocker)

   Stop a motor immediately, without deceleration ramp.
********************************************************************/
void HardStop(uint32_t Motor)
{
  VMaxModified[Motor]=TRUE;
  AMaxModified[Motor]=TRUE;
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_VMAX, 0);
  WriteTMC5130Int(WHICH_5130(Motor), TMC5130_AMAX, 65535);
  WriteTMC5130Datagram(WHICH_5130(Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELPOS);
}

/***************************************************************//**
   \fn ConvertVelocityUserToInternal(int UserVelocity)
   \brief Convert from pps to internal unit
   \param UserVelocity: Velocity as pps value
   \return Internal velocity value

  This function converts a velocity value given in pps for use
  with TMC5130 velocity registers.
********************************************************************/
int ConvertVelocityUserToInternal(int UserVelocity)
{
  if(UserVelocity>=0)
    return (int) floor((double) UserVelocity/VEL_FACTOR);
  else
    return (int) ceil((double) UserVelocity/VEL_FACTOR);
}


/***************************************************************//**
   \fn ConvertAccelerationUserToInternal(int UserAcceleration)
   \brief Convert from pps/s to internal unit
   \param UserAcceleration: Acceleration/Deceleration as pps/s value
   \return Internal acceleration/deceleration value

  This function converts an acceleration value or a deceleration value
  given in pps/s for use with TMC5130 acceleration/deceleration
  registers.
********************************************************************/
int ConvertAccelerationUserToInternal(int UserAcceleration)
{
  return (int) floor((double) UserAcceleration/ACC_FACTOR);
}


/***************************************************************//**
   \fn ConvertVelocityInternalToUser(int InternalVelocity)
   \brief Convert from internal unit to pps
   \param InternalVelocity: Velocity as internal value
   \return PPS velocity value

  This function converts a velocity value given in internal units
  of the TMC5130 back into pps.
********************************************************************/
int ConvertVelocityInternalToUser(int InternalVelocity)
{
  if(InternalVelocity>=0)
    return (int) ceil((double) InternalVelocity*VEL_FACTOR);
  else
    return (int) floor((double) InternalVelocity*VEL_FACTOR);
}

/***************************************************************//**
   \fn ConvertAccelerationInternalToUser(int InternalAcceleration)
   \brief Convert from internal unit to pps/s
   \param InternalAcceleration: Accleration/Deceleration as internal value
   \return PPS/S acceleration/deceleration value

  This function converts an acceleration/deceleration value given
  in internal units of the TMC5130 back into pps/s.
********************************************************************/
int ConvertAccelerationInternalToUser(int InternalAcceleration)
{
  return (int) ceil((float) InternalAcceleration*ACC_FACTOR);
}

/***************************************************************//**
   \fn ConvertInternalToInternal(int Internal)
   \brief Dummy function used when unit conversion is switched off
   \param Internal: Input value
   \return Unchanged value

   This is a dummy function which is used when unit conversion is
   switched off.
********************************************************************/
int ConvertInternalToInternal(int Internal)
{
  return Internal;
}
