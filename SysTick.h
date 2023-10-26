/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file SysTick.h ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: SysTick.h
 *         Description: Defitions for the 1ms system tick timer
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __SYS_TICK_H
#define __SYS_TICK_H

void InitSysTick(void);
uint32_t GetSysTimer(void);

#endif
