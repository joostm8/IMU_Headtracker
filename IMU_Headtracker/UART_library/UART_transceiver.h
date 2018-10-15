/*
 * UART_transceiver.h
 *
 * UART transceiver driver which is not reliant on circular buffer but
 * instead on traffic initiated from the microcontroller
 *
 * Created: 15/10/2018 15:20:10
 *  Author: Joost
 */ 


#ifndef UART_TRANSCEIVER_H_
#define UART_TRANSCEIVER_H_

/****************************************************************************
  UART Status/Control register definitions
****************************************************************************/

/****************************************************************************
  Global definitions
****************************************************************************/

/****************************************************************************
  Function definitions
****************************************************************************/
void UART_initialise();
uint8_t UART_tx(unsigned char*, uint32_t);
uint8_t UART_tx_rx(unsigned char*, unsigned char*, uint32_t);
uint8_t UART_tx_rx_w_timeout(unsigned char*, unsigned char*, uint32_t, uint32_t);
/****************************************************************************
  Bit and byte definitions
****************************************************************************/

/****************************************************************************
  UART State codes
****************************************************************************/

#endif /* UART_TRANSCEIVER_H_ */