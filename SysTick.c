/*******************************************************************************
* Copyright © 2021 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/

/** \file SysTick.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: SysTick.c
 *         Description: System Tick Timer
 *
 *
 *    Revision History:
 *                    2021_01_26    Rev 1.00    Olav Kahlbaum   File created
 *
 *  -------------------------------------------------------------------- */

#include "max32660.h"

static volatile uint32_t SysTickTimer;  //!< Millisecond counter


/***************************************************************//**
   \fn SysTick_Handler()
   \brief System Tick Timer Interrupt Handler

   Count up the 1ms counter. Gets called by the system tick interrupt.
********************************************************************/
void SysTick_Handler(void)
{
  mxc_delay_handler();  //to make mxc_delay work (if needed)

  SysTickTimer++;
}


/***************************************************************//**
   \fn InitSysTick()
   \brief Initialize system tick timer

   Initialize the system tick timer.
********************************************************************/
void InitSysTick(void)
{
  SysTickTimer=0;
  SysTick_Config(96000);  //96MHz => 1ms-Timer
}

/***************************************************************//**
   \fn GetSysTimer()
   \brief Read 1ms counter
   \return Milliseconds since startup.

   Returns the milliseconds that have expired since last reset.
********************************************************************/
uint32_t GetSysTimer(void)
{
  return SysTickTimer;
}
