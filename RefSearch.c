/** \file RefSearch.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: RefSearch.c
 *         Description: Reference Search using StallGuard for the Homebus slave
 *
 *
 *    Revision History:
 *                    2021_03_16    Rev 1.00    Olav Kahlbaum   File created
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
#include "HomebusSlave.h"
#include "Globals.h"
#include "TMC5130.h"

typedef enum {RFS_IDLE, RFS_START, RFS_WAITMAX, RFS_WAITMIN} TRefSearchState;

static TRefSearchState RefSearchState[N_O_MOTORS];
static uint32_t NormalStallVMin[N_O_MOTORS];
static int32_t NormalStallThreshold[N_O_MOTORS];
static uint32_t NormalGConfSetting[N_O_MOTORS];
static uint32_t NormalTCoolthrs[N_O_MOTORS];

void ProcessRefSearch(uint8_t axis)
{
  switch(RefSearchState[axis])
  {
    case RFS_IDLE:
      break;

    case RFS_START:
      if(AMaxModified[axis])
      {
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_AMAX, AMax[axis]);
        AMaxModified[axis]=FALSE;
      }
      VMaxModified[axis]=TRUE;
      StallFlag[axis]=FALSE;
      NormalStallVMin[axis]=StallVMin[axis];
      StallVMin[axis]=RefSearchStallVMin[axis];
      NormalStallThreshold[axis]=GetTMC5130SmartEnergyStallThreshold(axis);
      NormalGConfSetting[axis]=ReadTMC5130Int(WHICH_5130(axis), TMC5130_GCONF);
      WriteTMC5130Int(WHICH_5130(axis), TMC5130_GCONF, NormalGConfSetting[axis] & ~TMC5130_GCONF_EN_PWM_MODE);  //Switch off StealthChop during reference search
      NormalTCoolthrs[axis]=ReadTMC5130Int(WHICH_5130(axis), TMC5130_TCOOLTHRS);
      WriteTMC5130Int(WHICH_5130(axis), TMC5130_TCOOLTHRS, 1048757);
      SetTMC5130SmartEnergyStallThreshold(axis, RefSearchStallThreshold[axis]);
      WriteTMC5130Int(WHICH_5130(axis), TMC5130_VMAX, ConvertVelocityUserToInternal(abs(RefSearchVelocity[axis])));
      if(RefSearchVelocity[axis]>0)
        WriteTMC5130Datagram(WHICH_5130(axis), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELPOS);
      else
        WriteTMC5130Datagram(WHICH_5130(axis), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);

      RefSearchState[axis]=RFS_WAITMAX;
      break;

    case RFS_WAITMAX:
      if(StallFlag[axis])
      {
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_XTARGET, 0);
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_XACTUAL, 0);

        if(AMaxModified[axis])
        {
          WriteTMC5130Int(WHICH_5130(axis), TMC5130_AMAX, AMax[axis]);
          AMaxModified[axis]=FALSE;
        }
        VMaxModified[axis]=TRUE;
        StallFlag[axis]=FALSE;

        WriteTMC5130Int(WHICH_5130(axis), TMC5130_VMAX, ConvertVelocityUserToInternal(abs(RefSearchVelocity[axis])));
        if(RefSearchVelocity[axis]<0)  //Use opposite direction now
          WriteTMC5130Datagram(WHICH_5130(axis), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELPOS);
        else
          WriteTMC5130Datagram(WHICH_5130(axis), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);

        RefSearchState[axis]=RFS_WAITMIN;
      }
      break;

    case RFS_WAITMIN:
      if(StallFlag[axis])
      {
        RefSearchDistance[axis]=abs(ReadTMC5130Int(WHICH_5130(axis), TMC5130_XACTUAL));

        StallVMin[axis]=NormalStallVMin[axis];
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_GCONF, NormalGConfSetting[axis]);
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_TCOOLTHRS, NormalTCoolthrs[axis]);
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_XTARGET, 0);
        WriteTMC5130Int(WHICH_5130(axis), TMC5130_XACTUAL, 0);
        RefSearchState[axis]=RFS_IDLE;
      }
      break;
  }
}

void StartRefSearch(uint8_t axis)
{
  RefSearchState[axis]=RFS_START;
}

void StopRefSearch(uint8_t axis)
{
  if(RefSearchState[axis]!=RFS_IDLE)
  {
    VMaxModified[axis]=TRUE;
    WriteTMC5130Int(WHICH_5130(axis), TMC5130_VMAX, 0);
    WriteTMC5130Datagram(WHICH_5130(axis), TMC5130_RAMPMODE, 0, 0, 0, TMC5130_MODE_VELNEG);
    RefSearchState[axis]=RFS_IDLE;
    StallVMin[axis]=NormalStallVMin[axis];
    SetTMC5130SmartEnergyStallThreshold(axis, NormalStallThreshold[axis]);
  }
}

uint32_t GetRefSearchState(uint32_t axis)
{
  return (uint32_t) RefSearchState[axis];
}
