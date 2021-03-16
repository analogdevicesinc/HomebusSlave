/** \file TMCL.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: TMCL.c
 *         Description: TMCL interpreter for the homebus slave
 *                      (TMCL = Trinamic Motion Control Language)
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
#include "HomebusSlave.h"
#include "Globals.h"
#include "SysTick.h"
#include "TMC5130.h"
#include "TMCL.h"
#include "MAX31875.h"
#include "Homebus.h"
#include "RefSearch.h"

#define RS485_MODULE_ADDRESS 1
#define RS485_HOST_ADDRESS   2

extern const char VersionString[];

static uint8_t TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;            //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;                //!< Reply of last executed TMCL command
static uint8_t TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static uint8_t SpecialReply[9];               //!< buffer for special replies
static uint8_t ResetRequested;                //!< TRUE after executing the software reset command

static void RotateLeft(void);
static void RotateRight(void);
static void MotorStop(void);
static void MoveToPosition(void);
static void SetAxisParameter(void);
static void GetAxisParameter(void);
static void GetInput(void);
static void GetVersion(void);
static void ReferenceSearch(void);


void InitTMCL(void)
{
  uint32_t i;

  for(i=0; i<N_O_MOTORS; i++)
  {
    VMax[i]=ReadTMC5130Int(WHICH_5130(i), TMC5130_VMAX);
    VMaxModified[i]=FALSE;
    AMax[i]=ReadTMC5130Int(WHICH_5130(i), TMC5130_AMAX);
    AMaxModified[i]=FALSE;

    RefSearchVelocity[i]=100000;
    RefSearchStallVMin[i]=98000;
    RefSearchStallThreshold[i]=0;
  }
}


/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode=ActualCommand.Opcode;
  ActualReply.Status=REPLY_OK;
  ActualReply.Value.Int32=ActualCommand.Value.Int32;

  //Execute command
  switch(ActualCommand.Opcode)
  {
    case TMCL_ROR:
      RotateRight();
      break;

    case TMCL_ROL:
      RotateLeft();
      break;

    case TMCL_MST:
      MotorStop();
      break;

    case TMCL_MVP:
      MoveToPosition();
      break;

    case TMCL_SAP:
      SetAxisParameter();
      break;

    case TMCL_GAP:
      GetAxisParameter();
      break;

    case TMCL_GIO:
      GetInput();
      break;

    case TMCL_RFS:
      ReferenceSearch();
      break;

    case TMCL_GetVersion:
      GetVersion();
      break;

    default:
      ActualReply.Status=REPLY_INVALID_CMD;
      break;
  }
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  uint8_t RS485Cmd[9];
  uint8_t RS485Reply[9];
  uint8_t Checksum;
  uint32_t i;

  //**Send answer for the last command**
  if(TMCLCommandState==TCS_UART)  //via UART
  {
    if(TMCLReplyFormat==RF_STANDARD)
    {
      RS485Reply[8]=RS485_HOST_ADDRESS+RS485_MODULE_ADDRESS+
                    ActualReply.Status+ActualReply.Opcode+
                    ActualReply.Value.Byte[3]+
                    ActualReply.Value.Byte[2]+
                    ActualReply.Value.Byte[1]+
                    ActualReply.Value.Byte[0];

      RS485Reply[0]=RS485_HOST_ADDRESS;
      RS485Reply[1]=RS485_MODULE_ADDRESS;
      RS485Reply[2]=ActualReply.Status;
      RS485Reply[3]=ActualReply.Opcode;
      RS485Reply[4]=ActualReply.Value.Byte[3];
      RS485Reply[5]=ActualReply.Value.Byte[2];
      RS485Reply[6]=ActualReply.Value.Byte[1];
      RS485Reply[7]=ActualReply.Value.Byte[0];
      HomebusSendData(RS485Reply);
    }
    else if(TMCLReplyFormat==RF_SPECIAL)
    {
      HomebusSendData(SpecialReply);
    }
  }
  else if(TMCLCommandState==TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode=0;
    ActualReply.Status=REPLY_CHKERR;
    ActualReply.Value.Int32=0;

    RS485Reply[8]=RS485_HOST_ADDRESS+RS485_MODULE_ADDRESS+
                  ActualReply.Status+ActualReply.Opcode+
                  ActualReply.Value.Byte[3]+
                  ActualReply.Value.Byte[2]+
                  ActualReply.Value.Byte[1]+
                  ActualReply.Value.Byte[0];

    RS485Reply[0]=RS485_HOST_ADDRESS;
    RS485Reply[1]=RS485_MODULE_ADDRESS;
    RS485Reply[2]=ActualReply.Status;
    RS485Reply[3]=ActualReply.Opcode;
    RS485Reply[4]=ActualReply.Value.Byte[3];
    RS485Reply[5]=ActualReply.Value.Byte[2];
    RS485Reply[6]=ActualReply.Value.Byte[1];
    RS485Reply[7]=ActualReply.Value.Byte[0];
    HomebusSendData(RS485Reply);
  }

  //Reset state (answer has been sent now)
  TMCLCommandState=TCS_IDLE;
  TMCLReplyFormat=RF_STANDARD;

  //Generate a system reset if requested by the host
  //if(ResetRequested) ResetCPU(TRUE);

  //**Try to get a new command**
  if(HomebusGetData(RS485Cmd))  //Get data from UART
  {
    if(RS485Cmd[0]==RS485_MODULE_ADDRESS)  //Is this our addresss?
    {
      Checksum=0;
      for(i=0; i<8; i++) Checksum+=RS485Cmd[i];

      if(Checksum==RS485Cmd[8])  //Is the checksum correct?
      {
        ActualCommand.Opcode=RS485Cmd[1];
        ActualCommand.Type=RS485Cmd[2];
        ActualCommand.Motor=RS485Cmd[3];
        ActualCommand.Value.Byte[3]=RS485Cmd[4];
        ActualCommand.Value.Byte[2]=RS485Cmd[5];
        ActualCommand.Value.Byte[1]=RS485Cmd[6];
        ActualCommand.Value.Byte[0]=RS485Cmd[7];

        TMCLCommandState=TCS_UART;
      }
      else TMCLCommandState=TCS_UART_ERROR;  //Checksum wrong
    }
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if(TMCLCommandState!=TCS_IDLE && TMCLCommandState!=TCS_UART_ERROR) ExecuteActualCommand();
}


/***************************************************************//**
   \fn RotateLeft()
   \brief TMCL ROL command

   Execute TMCL ROL command.
********************************************************************/
static void RotateLeft(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    if(AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_AMAX, AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor]=FALSE;
    }
    VMaxModified[ActualCommand.Motor]=TRUE;
    StallFlag[ActualCommand.Motor]=FALSE;
    WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, ConvertVelocityUserToInternal(abs(ActualCommand.Value.Int32)));
    if(ActualCommand.Value.Int32>0)
      WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);
    else
      WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELPOS);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn RotateRight()
   \brief TMCL ROR command

   Execute TMCL ROR command.
********************************************************************/
static void RotateRight(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    if(AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_AMAX, AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor]=FALSE;
    }
    VMaxModified[ActualCommand.Motor]=TRUE;
    StallFlag[ActualCommand.Motor]=FALSE;
    WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, ConvertVelocityUserToInternal(abs(ActualCommand.Value.Int32)));
    if(ActualCommand.Value.Int32>0)
      WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELPOS);
    else
      WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn MotorStop()
   \brief TMCL MST command

   Execute TMCL MST command.
********************************************************************/
static void MotorStop(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    VMaxModified[ActualCommand.Motor]=TRUE;
    StallFlag[ActualCommand.Motor]=FALSE;
    WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, 0);
    WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn MoveToPosition()
   \brief TMCL MVP command

   Execute TMCL MVP command.
********************************************************************/
static void MoveToPosition(void)
{
  int NewPosition;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case MVP_ABS:
        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, VMax[ActualCommand.Motor]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        if(AMaxModified[ActualCommand.Motor])
        {
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_AMAX, AMax[ActualCommand.Motor]);
          AMaxModified[ActualCommand.Motor]=FALSE;
        }
        StallFlag[ActualCommand.Motor]=FALSE;
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XTARGET, ActualCommand.Value.Int32);
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_POSITION);
        break;

      case MVP_REL:
        if(VMaxModified[ActualCommand.Motor])
        {
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, VMax[ActualCommand.Motor]);
          VMaxModified[ActualCommand.Motor]=FALSE;
        }
        if(AMaxModified[ActualCommand.Motor])
        {
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_AMAX, AMax[ActualCommand.Motor]);
          AMaxModified[ActualCommand.Motor]=FALSE;
        }
        StallFlag[ActualCommand.Motor]=FALSE;
        NewPosition=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XTARGET)+ActualCommand.Value.Int32;
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XTARGET, NewPosition);
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_POSITION);
        ActualReply.Value.Int32=NewPosition;
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetAxisParameter()
   \brief TMCL SAP command

   Execute TMCL SAP command.
********************************************************************/
void SetAxisParameter(void)
{
  uint32_t Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case 0:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XTARGET, ActualCommand.Value.Int32);
        break;

      case 1:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XACTUAL, ActualCommand.Value.Int32);
        break;

      case 2:
        if(ActualCommand.Value.Int32>0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, TMC5130_MODE_VELPOS);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE, TMC5130_MODE_VELNEG);

        VMaxModified[ActualCommand.Motor]=TRUE;
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, ConvertVelocityUserToInternal(abs(ActualCommand.Value.Int32)));
        break;

      case 4:
        VMax[ActualCommand.Motor]=ConvertVelocityUserToInternal(abs(ActualCommand.Value.Int32));
        if(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE)==TMC5130_MODE_POSITION)
        {
            WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX, VMax[ActualCommand.Motor]);
        }
        break;

      case 5:
        AMaxModified[ActualCommand.Motor]=FALSE;
        AMax[ActualCommand.Motor]=ConvertAccelerationUserToInternal(ActualCommand.Value.Int32);
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_AMAX, AMax[ActualCommand.Motor]);
        break;

      case 6:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN, 0, Value >> 16,
                            ActualCommand.Value.Byte[0]/8, Value & 0xff);
        break;

      case 7:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN, 0, Value >> 16,
                            Value >> 8, ActualCommand.Value.Byte[0]/8);
        break;

      case 12:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32==0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_STOPR_ENABLE);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_STOPR_ENABLE);
        break;

      case 13:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32==0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_STOPL_ENABLE);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_STOPL_ENABLE);
        break;

      case 14:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_SWAP_LR);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_SWAP_LR);
        break;

      case 15:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_A1, ConvertAccelerationUserToInternal(ActualCommand.Value.Int32));
        break;

      case 16:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_V1, ConvertVelocityUserToInternal(ActualCommand.Value.Int32));
        break;

      case 17:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DMAX, ConvertAccelerationUserToInternal(ActualCommand.Value.Int32));
        break;

      case 18:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_D1, ConvertAccelerationUserToInternal(ActualCommand.Value.Int32));
        break;

      case 19:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VSTART, ConvertVelocityUserToInternal(ActualCommand.Value.Int32));
        break;

      case 20:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VSTOP, ConvertVelocityUserToInternal(ActualCommand.Value.Int32));
        break;

      case 21:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TZEROWAIT, ActualCommand.Value.Int32);
        break;

      case 22:
        if(ActualCommand.Value.Int32>=0)
        {
          if(ActualCommand.Value.Int32>0)
            WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_THIGH, 13000000 / ActualCommand.Value.Int32);
          else
            WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_THIGH, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 23:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VDCMIN, ConvertVelocityUserToInternal(ActualCommand.Value.Int32));
        break;

      case 24:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_STOPR_POLARITY);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_STOPR_POLARITY);
        break;

      case 25:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_STOPL_POLARITY);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_STOPL_POLARITY);
        break;

      case 26:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value|TMC5130_SW_SOFTSTOP);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE, Value & ~TMC5130_SW_SOFTSTOP);
        break;

      case 27:
        SetTMC5130ChopperVHighChm(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 28:
        SetTMC5130ChopperVHighFs(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 31:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN, 0, ActualCommand.Value.Byte[0] & 0x0f,
                             Value >> 8, Value & 0xff);
        break;

      case 32:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL);  //DC_TIME
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL, 0, Value >> 16,
                             ActualCommand.Value.Byte[1] & 0x03, ActualCommand.Value.Byte[0]);
        break;

      case 33:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL);  //DC_SG
        WriteTMC5130Datagram(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL, 0, ActualCommand.Value.Byte[0],
                             Value >> 8, Value & 0xff);
        break;

      case 140:
        SetTMC5130ChopperMStepRes(ActualCommand.Motor, 8-ActualCommand.Value.Int32);
        break;

      case 167:
        SetTMC5130ChopperTOff(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 168:
        SetTMC5130SmartEnergyIMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 169:
        SetTMC5130SmartEnergyDownStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 170:
        SetTMC5130SmartEnergyStallLevelMax(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 171:
        SetTMC5130SmartEnergyUpStep(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 172:
        SetTMC5130SmartEnergyStallLevelMin(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 173:
        SetTMC5130SmartEnergyFilter(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 174:
        SetTMC5130SmartEnergyStallThreshold(ActualCommand.Motor, ActualCommand.Value.Int32);
        break;

      case 179:
        SetTMC5130ChopperVSenseMode(ActualCommand.Motor, ActualCommand.Value.Byte[0]);
        break;

      case 181:
        StallVMin[ActualCommand.Motor]=ConvertVelocityUserToInternal(ActualCommand.Value.Int32);
        break;

      case 182:
        if(ActualCommand.Value.Int32>=0)
        {
          if(ActualCommand.Value.Int32>0)
            WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TCOOLTHRS, 12500000 / ActualCommand.Value.Int32);
          else
            WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TCOOLTHRS, 1048757);
        }
        else ActualReply.Status=REPLY_INVALID_VALUE;
        break;

      case 193:
        RefSearchStallThreshold[ActualCommand.Motor]=ActualCommand.Value.Int32;
        break;

      case 194:
        RefSearchVelocity[ActualCommand.Motor]=ActualCommand.Value.Int32;
        break;

      case 195:
        RefSearchStallVMin[ActualCommand.Motor]=ActualCommand.Value.Int32;
        break;

      case 214:
        WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TPOWERDOWN, (int) floor((double) ActualCommand.Value.Int32/TPOWERDOWN_FACTOR));
        break;

      case 251:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_GCONF);
        if(ActualCommand.Value.Int32!=0)
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_GCONF, Value|TMC5130_GCONF_SHAFT);
        else
          WriteTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_GCONF, Value & ~TMC5130_GCONF_SHAFT);
        break;

       default:
        ActualReply.Status=REPLY_WRONG_TYPE;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn GetAxisParameter()
   \brief TMCL GAP command

   Execute TMCL GAP command.
********************************************************************/
void GetAxisParameter(void)
{
  uint32_t Value;

  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case 0:
        ActualReply.Value.Int32=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XTARGET);
        break;

      case 1:
        ActualReply.Value.Int32=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_XACTUAL);
        break;

      case 2:
        if(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPMODE)==TMC5130_MODE_VELPOS)
          ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX));
        else
          ActualReply.Value.Int32=-ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VMAX));
        break;

      case 3:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VACTUAL));
        break;

      case 4:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(VMax[ActualCommand.Motor]);
        break;

      case 5:
        ActualReply.Value.Int32=ConvertAccelerationInternalToUser(AMax[ActualCommand.Motor]);
        break;

      case 6:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        ActualReply.Value.Int32=((Value>>8) & 0xff)*8;
        break;

      case 7:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        ActualReply.Value.Int32=(Value & 0xff)*8;
        break;

      case 8:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPSTAT) & TMC5130_RS_POSREACHED) ? 1:0;
        break;

      case 10:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPSTAT) & TMC5130_RS_STOPR) ? 1:0;
        break;

      case 11:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_RAMPSTAT) & TMC5130_RS_STOPL) ? 1:0;
        break;

      case 12:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_STOPR_ENABLE) ? 0:1;
        break;

      case 13:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_STOPL_ENABLE) ? 0:1;
        break;

      case 14:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_SWAP_LR) ? 1:0;
        break;

      case 15:
        ActualReply.Value.Int32=ConvertAccelerationInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_A1));
        break;

      case 16:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_V1));
        break;

      case 17:
        ActualReply.Value.Int32=ConvertAccelerationInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DMAX));
        break;

      case 18:
        ActualReply.Value.Int32=ConvertAccelerationInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_D1));
        break;

      case 19:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VSTART));
        break;

      case 20:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VSTOP));
        break;

      case 21:
        ActualReply.Value.Int32=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TZEROWAIT);
        break;

      case 22:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_THIGH);
        if(Value>0)
          ActualReply.Value.Int32=16000000/Value;
         else
          ActualReply.Value.Int32=16777215;
        break;

      case 23:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_VDCMIN));
        break;

      case 24:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_STOPR_POLARITY) ? 1:0;
        break;

      case 25:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_STOPL_POLARITY) ? 1:0;
        break;

      case 26:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_SWMODE) & TMC5130_SW_SOFTSTOP) ? 1:0;
        break;

      case 27:
        ActualReply.Value.Int32=GetTMC5130ChopperVHighChm(ActualCommand.Motor);
        break;

      case 28:
        ActualReply.Value.Int32=GetTMC5130ChopperVHighFs(ActualCommand.Motor);
        break;

      case 30:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        ActualReply.Value.Int32=((Value>>16) & 0x0f);
        break;

      case 31:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_IHOLD_IRUN);
        ActualReply.Value.Int32=((Value>>16) & 0x0f);
        break;

      case 32:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL);
        ActualReply.Value.Int32=Value & 0x3ff;
        break;

      case 33:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DCCTRL);
        ActualReply.Value.Int32=Value >> 16;
        break;

      case 140:
        ActualReply.Value.Int32=8-GetTMC5130ChopperMStepRes(ActualCommand.Motor);
        break;

      case 167:
        ActualReply.Value.Int32=GetTMC5130ChopperTOff(ActualCommand.Motor);
        break;

      case 168:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyIMin(ActualCommand.Motor);
        break;

      case 169:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyDownStep(ActualCommand.Motor);
        break;

      case 170:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyStallLevelMax(ActualCommand.Motor);
        break;

      case 171:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyUpStep(ActualCommand.Motor);
        break;

      case 172:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyStallLevelMin(ActualCommand.Motor);
        break;

      case 173:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyFilter(ActualCommand.Motor);
        break;

      case 174:
        ActualReply.Value.Int32=GetTMC5130SmartEnergyStallThreshold(ActualCommand.Motor);
        break;

      case 179:
        ActualReply.Value.Int32=GetTMC5130ChopperVSenseMode(ActualCommand.Motor);
        break;

      case 180:
        ActualReply.Value.Int32=(ReadTMC5130Int(ActualCommand.Motor, TMC5130_DRVSTATUS) >> 16) & 0x1f;
        break;

      case 181:
        ActualReply.Value.Int32=ConvertVelocityInternalToUser(StallVMin[ActualCommand.Motor]);
        break;

      case 182:
        Value=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TCOOLTHRS);
        if(Value>0)
          ActualReply.Value.Int32=12500000/Value;
         else
          ActualReply.Value.Int32=16777215;
        break;

      case 193:
        ActualReply.Value.Int32=RefSearchStallThreshold[ActualCommand.Motor];
        break;

      case 194:
        ActualReply.Value.Int32=RefSearchVelocity[ActualCommand.Motor];
        break;

      case 195:
        ActualReply.Value.Int32=RefSearchStallVMin[ActualCommand.Motor];
        break;

      case 206:
        ActualReply.Value.Int32=ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DRVSTATUS) & 0x3ff;
        break;

      case 207:
        ActualReply.Value.Int32=StallFlag[ActualCommand.Motor];
        break;

      case 208:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_DRVSTATUS) >> 24) & 0xff;
        break;

      case 214:
        ActualReply.Value.Int32=(int) ceil((double) ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_TPOWERDOWN)*TPOWERDOWN_FACTOR);
        break;

      case 251:
        ActualReply.Value.Int32=(ReadTMC5130Int(WHICH_5130(ActualCommand.Motor), TMC5130_GCONF) & TMC5130_GCONF_SHAFT) ? 1:0;
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  } else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn GetInput()
   \brief TMCL GIO command

   Execute TMCL GIO command.
********************************************************************/
static void GetInput(void)
{
  switch(ActualCommand.Motor)
  {
    case 1:
      switch(ActualCommand.Type)
      {
        case 9:
          ActualReply.Value.Int32=GetTemperature();
          break;

        default:
          ActualReply.Status=REPLY_WRONG_TYPE;
          break;
      }
      break;

    default:
      ActualReply.Status=REPLY_INVALID_VALUE;
      break;
  }
}


/***************************************************************//**
   \fn ReferenceSearch()
   \brief TMCL RFS command

   Execute TMCL RFS command.
********************************************************************/
static void ReferenceSearch(void)
{
  if(ActualCommand.Motor<N_O_MOTORS)
  {
    switch(ActualCommand.Type)
    {
      case RFS_START:
        StartRefSearch(ActualCommand.Motor);
        break;

      case RFS_STOP:
        StopRefSearch(ActualCommand.Motor);
        break;

      case RFS_STATUS:
        ActualReply.Value.Int32=GetRefSearchState(ActualCommand.Motor);
        break;

      default:
        ActualReply.Status=REPLY_WRONG_TYPE;
        break;
    }
  }
  else ActualReply.Status=REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  uint32_t i;

  switch(ActualCommand.Type)
  {
    case 0:
      TMCLReplyFormat=RF_SPECIAL;
      SpecialReply[0]=RS485_HOST_ADDRESS;
      for(i=0; i<8; i++)
        SpecialReply[i+1]=VersionString[i];
      break;

    case 1:
      ActualReply.Value.Byte[3]=SW_TYPE_HIGH;
      ActualReply.Value.Byte[2]=SW_TYPE_LOW;
      ActualReply.Value.Byte[1]=SW_VERSION_HIGH;
      ActualReply.Value.Byte[0]=SW_VERSION_LOW;
      break;

    default:
      ActualReply.Status=REPLY_WRONG_TYPE;
      break;
  }
}
