/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

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
