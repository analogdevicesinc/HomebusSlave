/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file HomebusSlave.h *******************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: HomebusSlave.h
 *         Description: Some common definitions for the Homebus slave
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __HOMEBUS_SLAVE_H
#define __HOMEBUS_SLAVE_H

#define SW_VERSION_HIGH 1
#define SW_VERSION_LOW  0
#define SW_TYPE_HIGH 0x00
#define SW_TYPE_LOW  0x1A

#define WHICH_5130(a) (a)

#define N_O_MOTORS 1
#define N_O_DRIVERS 1

#endif
