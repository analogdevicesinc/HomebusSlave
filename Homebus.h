/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file Homebus.h ***********************************************************
 * 
 *             Project: Homebus Reference Design
 *            Filename: Homebus.h
 *         Description: Homebus interface functions
 *                   
 * 
 *    Revision History:
 *                    2021_01_26		Rev 1.00    Olav Kahlbaum		File created
 *
 *  -------------------------------------------------------------------- */
  
#ifndef __HOMEBUS_H
#define __HOMEBUS_H

void HomebusInit(uint32_t Baudrate);
uint8_t HomebusGetData(uint8_t *data);
void HomebusSendData(uint8_t *data);

#endif
