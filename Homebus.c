/** \file Homebus.c ***********************************************************
 *
 *             Project: Homebus Reference Design
 *            Filename: Homebus.c
 *         Description: Handling of the Homebus interface
 *                      (send and receive TMCL commands via homebus)
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
#include "bits.h"
#include "mxc_sys.h"
#include "uart.h"
#include "gpio.h"

#define TMCL_COMMAND_LENGTH      9                           //!< Length of a TMCL command: 9 bytes
#define HBS_TMCL_COMMAND_LENGTH  2*TMCL_COMMAND_LENGTH       //!< Length of a homebus encoder TMCL command
#define HBS_RX_THRESHOLD         3     //!< Threshold value for Rx FIFO. HBS_TMCL_COMMAND_LENGTH must be dividable by HBS_RX_THRESHOLD.

static const sys_cfg_uart_t sys_uart0_cfg = {
        MAP_A,
        UART_FLOW_DISABLE,
};

static gpio_cfg_t HomebusTxPin;  //!< Pin for switching between send and receive mode (MAX22088 RST pin)
static uint8_t HomebusRawRxData[HBS_TMCL_COMMAND_LENGTH];   //!< Buffer for incoming homebus data
static uint8_t HomebusRawTxData[HBS_TMCL_COMMAND_LENGTH];   //!< Buffer for outgoing homebus data
static volatile uint8_t HomebusRawRxCount;                           //!< Counter for incoming homebus data
static uint8_t Command[TMCL_COMMAND_LENGTH];                //!< Buffer for received TMCL command


/***************************************************************//**
   \fn Homebus_data_decode()
   \param rx_raw_data: pointer to homebus data
   \param rx_data: pointer to decoded data
   \param count: number of bytes to be decoded
   \brief Decode homebus encoded data

   Decode homebus data that has been encoded by the sender using
   the function homebus_data_encode.
********************************************************************/
void Homebus_data_decode(uint8_t *rx_raw_data, uint8_t *rx_data, uint8_t count)
{
  for (int i = 0; i < count; i++)
  {
    rx_data[i] = ( (rx_raw_data[(i*2)+0] & 0x80) >>4 ) +
                 ( (rx_raw_data[(i*2)+0] & 0x20) >>3 ) +
                 ( (rx_raw_data[(i*2)+0] & 0x08) >>2 ) +
                 ( (rx_raw_data[(i*2)+0] & 0x02) >>1 ) +
                 ( (rx_raw_data[(i*2)+1] & 0x80) <<0 ) +
                 ( (rx_raw_data[(i*2)+1] & 0x20) <<1 ) +
                 ( (rx_raw_data[(i*2)+1] & 0x08) <<2 ) +
                 ( (rx_raw_data[(i*2)+1] & 0x02) <<3 );
  }
}


/***************************************************************//**
   \fn Homebus_data_encode()
   \param tx_raw_data: pointer to encoded data
   \param tx_data: pointer to data to be encoded
   \param count: number of bytes to be encoded
   \brief Encode data for Homebus

   Encode data in such a way that there is always at least one 1-bit
   after each 0-bit. This is not fully HBS compliant as we are not
   using a parity bit, but can be used to transmit UART data via a
   homebus interface.
********************************************************************/
void Homebus_data_encode(uint8_t *tx_raw_data, uint8_t *tx_data, uint8_t count)
{
  for (int i=0; i < count; i++)
  {
    tx_raw_data[(i*2) +0] = ( (tx_data[i] & 0x08) <<4 ) +
                            ( (tx_data[i] & 0x04) <<3 ) +
                            ( (tx_data[i] & 0x02) <<2 ) +
                            ( (tx_data[i] & 0x01) <<1 ) + 0x55;

    tx_raw_data[(i*2) +1] = ( (tx_data[i] & 0x80) >>0 ) +
                            ( (tx_data[i] & 0x40) >>1 ) +
                            ( (tx_data[i] & 0x20) >>2 ) +
                            ( (tx_data[i] & 0x10) >>3 ) + 0x55;
  }
}


/***************************************************************//**
   \fn UART0_IRQHandler
   \brief UART 0 interrupt handler

   Handler function for the UART0 interrupt. UART0 is connected to
   the MAX22088 homebus transceiver on the reference design.
********************************************************************/
void UART0_IRQHandler(void)
{
  uint32_t irq_flags;
  uint8_t i;

  irq_flags=MXC_UART0->int_fl;

  //Recive threshold interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_FIFO_THRESH)
  {
    //The configured number of bytes have been received by the UART Rx FIFO.
    //Copy these bytes from the FIFO into our Homebus data buffer.
    for(i=0; i<HBS_RX_THRESHOLD; i++)
    {
      if(HomebusRawRxCount<HBS_TMCL_COMMAND_LENGTH) HomebusRawRxData[HomebusRawRxCount++]=MXC_UART0->fifo;
    }

    if(HomebusRawRxCount==HBS_TMCL_COMMAND_LENGTH)
    {
      //Entire TMCL command received => decode the data
      Homebus_data_decode(HomebusRawRxData, Command, TMCL_COMMAND_LENGTH);
    }

    //Reset the interrupt
    MXC_UART0->int_fl=MXC_F_UART_INT_FL_RX_FIFO_THRESH;
  }

  //Receive timeout interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_TIMEOUT)
  {
    //A timeout has occured while receiving => discard everything.
    MXC_UART0->int_fl=MXC_F_UART_INT_FL_RX_TIMEOUT;
    MXC_UART0->ctrl |= MXC_F_UART_CTRL_RX_FLUSH;
    HomebusRawRxCount=0;
  }

  //Receive FIFO overrun interrupt
  if(irq_flags & MXC_F_UART_INT_FL_RX_OVERRUN)
  {
    //An Rx FIFO overrun has occured (hardly possible) => discard everything
    MXC_UART0->int_fl=MXC_F_UART_INT_FL_RX_OVERRUN;
    MXC_UART0->ctrl |= MXC_F_UART_CTRL_RX_FLUSH;
    HomebusRawRxCount=0;
  }

  //Transmit interrupt
  if(irq_flags & (MXC_F_UART_INT_FL_TX_FIFO_THRESH|MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY))
  {
    //Has the very last bit been sent out?
    if((MXC_UART0->status & MXC_F_UART_STATUS_TX_EMPTY) && !(MXC_UART0->status & MXC_F_UART_STATUS_TX_BUSY))
    {
      //Yes: switch back MAX22088 transceiver to receive mode
      GPIO_OutSet(&HomebusTxPin);

      //Reset and disable the transmit interrupts
      MXC_UART0->int_en&= ~(MXC_F_UART_INT_EN_TX_FIFO_THRESH|MXC_F_UART_INT_EN_TX_FIFO_ALMOST_EMPTY);
      MXC_UART0->int_fl|=MXC_F_UART_INT_FL_TX_FIFO_THRESH|MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY;

      //Switch on the UART RX pin again
      MXC_GPIO0->en&= ~BIT5;
      MXC_GPIO0->en1|=BIT5;
    }
  }
}


/***************************************************************//**
   \fn HomebusInit()
   \param Baudrate: Baud rate to be used (mostly 230400)
   \brief Initalize everything for Homebus communication

   Initialize the Homebus communication.
********************************************************************/
void HomebusInit(uint32_t Baudrate)
{
  uart_cfg_t cfg;
  gpio_cfg_t rxIn;

  //Initialize port pin P0.6 for switching between transmit and receive
  //mode. This pin is connected to the MAX22088 RST pin.
  HomebusTxPin.port = PORT_0;
  HomebusTxPin.mask = PIN_6;
  HomebusTxPin.pad = GPIO_PAD_NONE;
  HomebusTxPin.func = GPIO_FUNC_OUT;
  GPIO_Config(&HomebusTxPin);
  GPIO_OutSet(&HomebusTxPin);  //Receive Mode

  //Prepare the UART0 RxD pin as input with pull-up
  //for switching off UART RX later (to suppress the echo from the bus).
  rxIn.port = PORT_0;
  rxIn.mask = PIN_5;
  rxIn.pad = GPIO_PAD_PULL_UP;
  rxIn.func = GPIO_FUNC_IN;
  GPIO_Config(&rxIn);

  //Initialize UART0
  cfg.parity = UART_PARITY_DISABLE;
  cfg.size = UART_DATA_SIZE_8_BITS;
  cfg.stop = UART_STOP_1;
  cfg.flow = UART_FLOW_CTRL_DIS;
  cfg.pol = UART_FLOW_POL_EN;
  cfg.baud = Baudrate;
  UART_Init(MXC_UART0, &cfg, &sys_uart0_cfg);
  MXC_UART0->ctrl|=(5<<16);  //Timeout: 5 Frames

  //Configure UART0 interrupts
  MXC_UART0->int_fl=0xffffffff;
  NVIC_ClearPendingIRQ(UART0_IRQn);
  NVIC_DisableIRQ(UART0_IRQn);
  NVIC_SetPriority(UART0_IRQn, 2);
  NVIC_EnableIRQ(UART0_IRQn);
  MXC_UART0->thresh_ctrl=HBS_RX_THRESHOLD;
  MXC_UART0->int_en=MXC_F_UART_INT_EN_RX_FIFO_THRESH|MXC_F_UART_INT_EN_RX_OVERRUN|MXC_F_UART_INT_EN_RX_TIMEOUT;
  MXC_UART0->ctrl|=MXC_F_UART_CTRL_TX_FLUSH|MXC_F_UART_CTRL_RX_FLUSH;
}


/***************************************************************//**
   \fn HomebusSendData()
   \param *data: pointer to TMCL command or reply (9 bytes)
   \brief Send TMCL data (command or reply) via Homebus

   Sends a TMCL command or a TMCL reply (consisting of 9 bytes) via
   Homebus. The echo will be suppressed by switching off the UART Rx
   pin as long as the sending goes.
   Switching back to receive mode is done in the interrupt handler.
********************************************************************/
void HomebusSendData(uint8_t *data)
{
  uint8_t i;

  //Switch off UART0 Rx pin (to suppress the echo)
  MXC_GPIO0->en|=BIT5;
  MXC_GPIO0->en1&= ~BIT5;

  //Switch MAX22088 transceiver to transmit mode
  GPIO_OutClr(&HomebusTxPin);

  //Enable transmit interrupts
  MXC_UART0->int_fl=MXC_F_UART_INT_FL_TX_FIFO_ALMOST_EMPTY;
  MXC_UART0->int_en|=MXC_F_UART_INT_EN_TX_FIFO_ALMOST_EMPTY;

  //Encode the data for Homebus
  Homebus_data_encode(HomebusRawTxData, data, TMCL_COMMAND_LENGTH);

  //Send out the data
  for(i=0; i<HBS_TMCL_COMMAND_LENGTH; i++)
  {
    while(MXC_UART0->status & MXC_F_UART_STATUS_TX_FULL);
    MXC_UART0->fifo=HomebusRawTxData[i];
  }
}


/***************************************************************//**
   \fn HomebusGetData()
   \param *data: pointer to array for received data (9 bytes)

   \return  TRUE if data could be read\n
            FALSE if no data available.

   \brief Get TMCL command or reply

   Read a TMCL command or a TMCL reply from the homebus interface.
********************************************************************/
uint8_t HomebusGetData(uint8_t *data)
{
  uint8_t i;

  if(HomebusRawRxCount==HBS_TMCL_COMMAND_LENGTH)
  {
    __disable_irq();
    for(i=0; i<TMCL_COMMAND_LENGTH; i++) data[i]=Command[i];
    HomebusRawRxCount=0;
    __enable_irq();

    return TRUE;
  }

  return FALSE;
}
