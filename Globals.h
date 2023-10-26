/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file Globals.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: Globals.h
 *         Description: Some global variables for the Homebus slave
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __GLOBALS_H
#define __GLOBALS_H

extern uint32_t VMax[N_O_MOTORS];
extern uint8_t VMaxModified[N_O_MOTORS];
extern int AMax[N_O_MOTORS];
extern uint8_t AMaxModified[N_O_MOTORS];
extern uint8_t StallFlag[N_O_MOTORS];
extern uint32_t StallVMin[N_O_MOTORS];
extern int32_t RefSearchStallThreshold[N_O_MOTORS];
extern int32_t RefSearchVelocity[N_O_MOTORS];
extern uint32_t RefSearchStallVMin[N_O_MOTORS];
extern int RefSearchDistance[N_O_MOTORS];

#endif
