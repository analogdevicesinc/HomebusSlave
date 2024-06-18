/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file MAX31875.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: MAX31875.h
 *         Description: Handling of the Homebus interface
 *                      (send and receive TMCL commands via homebus)
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __MAX31875_H
#define __MAX31875_H

void InitMAX31875(void);
int GetTemperature(void);

#endif
