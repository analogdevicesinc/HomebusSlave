/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file RefSearch.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: RefSearch.h
 *         Description: Reference Search using StallGuard for the Homebus slave
 *
 *
 *    Revision History:
 *                    2021_03_16    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#ifndef __REF_SEARCH_H
#define __REF_SEARCH_H

void ProcessRefSearch(uint8_t axis);
void StartRefSearch(uint8_t axis);
void StopRefSearch(uint8_t axis);
uint32_t GetRefSearchState(uint8_t axis);

#endif
