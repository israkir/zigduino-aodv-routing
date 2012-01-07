/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Contributing Authors (specific to this file):
*  Chipcon Development Team
*  Anthony Rowe
*******************************************************************************/



/*******************************************************************************************************
 * INSTRUCTIONS:                                                                                       *
 * Startup:                                                                                            *
 *     1. Create a RF_RX_INFO structure, and initialize the following members:                         *
 *         - rfRxInfo.pPayload (must point to an array of at least RF_MAX_PAYLOAD_SIZE bytes)          *
 *     2. Call rf_init() to initialize the packet protocol.                                            *
 *                                                                                                     *
 * Transmission:                                                                                       *
 *     1. Create a RF_TX_INFO structure, and initialize the following members:                         *
 *         - rfTxInfo.destAddr (the destination address, on the same PAN as you)                       *
 *         - rfTxInfo.pPayload (the payload data to be transmitted to the other node)                  *
 *         - rfTxInfo.length (the size od rfTxInfo.pPayload)                                           *
 *         - rfTxInfo.ackRequest (acknowledgment requested)                                            *
 *     2. Call rf_tx_packet()                                                                          *
 *                                                                                                     *
 * Reception:                                                                                          *
 *     1. Call rf_rx_on() to enable packet reception                                                   *
 *     2. When a packet arrives, the FIFOP interrupt will run, and will in turn call                   *
 *        rf_rx_callback(), which must be defined by the application however could be left blank       *
 *     3. Instead of the callback, you can call rf_rx_packet() and see if it returns 1 or a 0          *
 *        a 1 indicates that a packet is ready and can be directly accessed                            *
 *     3. Call rf_rx_off() to disable packet reception                                                 *
 *                                                                                                     *
 * FRAME FORMATS:                                                                                      *
 * Data packets:                                                                                       *
 *     [Preambles (4)][SFD (1)][Length (1)][Frame control field (2)][Sequence number (1)][PAN ID (2)]  *
 *     [Dest. address (2)][Source address (2)][Payload (Length - 2+1+2+2+2)][Frame check sequence (2)] *
 *                                                                                                     *
 * Acknowledgment packets:                                                                             *
 *     [Preambles (4)][SFD (1)][Length = 5 (1)][Frame control field (2)][Sequence number (1)]          *
 *     [Frame check sequence (2)]                                                                      *
 *******************************************************************************************************
 * Compiler: AVR-GCC                                                                                   *
 * Target platform: CC2420DB, CC2420 + any MCU with very few modifications required                    *
 *******************************************************************************************************/
#ifndef BASIC_RF_H
#define BASIC_RF_H

#define CHECKSUM_OVERHEAD	1

#include <stdbool.h>
#include <nrk_events.h>



void rf_auto_ack_enable();
void rf_auto_ack_disable();
void rf_addr_decode_set_my_mac(uint16_t my_mac);
void rf_addr_decode_enable();
void rf_addr_decode_disable();

/* NOT IMPLEMENTED
void halRfWaitForCrystalOscillator(void);
void halRfSetChannel(uint8_t channel);

uint8_t rf_security_last_pkt_status();
void rf_security_set_key(uint8_t *key);
void rf_security_set_ctr_counter(uint8_t *counter);
void rf_security_enable();
void rf_security_disable();

nrk_sem_t* rf_get_sem();
*/


/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 General constants                 **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//-------------------------------------------------------------------------------------------------------
// Constants concerned with the Basic RF packet format
// Packet overhead ((frame control field, sequence number, PAN ID, destination and source) + (footer))
// Note that the length byte itself is not included included in the packet length
#define RF_PACKET_OVERHEAD_SIZE   ((2 + 1 + 2 + 2 + 2) + (2))
#define RF_MAX_PAYLOAD_SIZE		(127 - RF_PACKET_OVERHEAD_SIZE)
#define RF_ACK_PACKET_SIZE		5

// The time it takes for the acknowledgment packet to be received after the data packet has been
// transmitted
#define RF_ACK_DURATION			(0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1) + (2)))
#define RF_SYMBOL_DURATION	    (32 * 0.5)

// The length byte
#define RF_LENGTH_MASK            0x7F

// Frame control field

#define RF_FCF_NOACK              0x8841
#define RF_FCF_ACK                0x8861

#define RF_SEC_BM		  0x0008
#define RF_ACK_BM		  0x0020
#define RF_FCF_ACK_BM             0x0020
#define RF_FCF_BM               (~RF_FCF_ACK_BM)
#define RF_ACK_FCF		        0x0002

// Footer
#define RF_CRC_OK_BM              0x80
//-------------------------------------------------------------------------------------------------------




/*******************************************************************************************************
 *******************************************************************************************************
 **************************                Packet transmission                **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//-------------------------------------------------------------------------------------------------------
// The data structure which is used to transmit packets
typedef struct {
//	uint16_t destPanId;
  uint16_t destAddr;
  int8_t length;
      uint8_t *pPayload;
  bool cca;
  bool ackRequest;
} RF_TX_INFO;
//-------------------------------------------------------------------------------------------------------

void rf_power_up();
void rf_power_down();


/* NOT IMPLEMENTED
void rf_test_mode();
void rf_data_mode();
void rf_rx_set_serial();
void rf_tx_set_serial();
void rf_flush_rx_fifo();
void rf_carrier_on();
void rf_carrier_off();
void rf_set_cca_thresh(int8_t t);
*/


//-------------------------------------------------------------------------------------------------------
//  bool rf_tx_packet(RF_TX_INFO *pRTI)
//
//  DESCRIPTION:
//		Transmits a packet with a short addresses. CCA is
//		measured only once before packet transmission (not compliant with 802.15.4 CSMA-CA).
//		The function returns:
//			- When pRTI->ackRequest is FALSE: After the transmission has begun (SFD gone high)
//			- When pRTI->ackRequest is TRUE: After the acknowledgment has been received/declared missing.
//		The acknowledgment is received through the FIFOP interrupt.
//
//  ARGUMENTS:
//      RF_TX_INFO *pRTI
//          The transmission structure, which contains all relevant info about the packet.
//
//  RETURN VALUE:
//		bool
//			Successful transmission (acknowledgment received)
//-------------------------------------------------------------------------------------------------------

uint8_t rf_tx_packet(RF_TX_INFO *pRTI);
uint8_t rf_tx_packet_repeat(RF_TX_INFO *pRTI, uint16_t ms);
int8_t rf_cca_check();

/* NOT IMPLEMENTED
uint8_t rf_tx_tdma_packet(RF_TX_INFO *pRTI, uint16_t slot_start_time, uint16_t tx_guard_time); 
void rf_set_channel(uint8_t channel);
*/


//-------------------------------------------------------------------------------------------------------
//  void rf_tx_power(uint8_t powerwdrf_tx_packet)
//
//  DESCRIPTION:
//
//	This function sets the power of the transmitter.  It accepts a value between 0 and 32
//			0 = -25 db
//			32 = 0 db
//
//  ARGUMENTS:
//      uint8_t power 
//          Contains a value ranging from 0 to 32 that should be set as the tx power 
//
//  RETURN VALUE:
//
//-------------------------------------------------------------------------------------------------------
void rf_tx_power(uint8_t pwr);



/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 Packet reception                  **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//-------------------------------------------------------------------------------------------------------
// The receive struct:
typedef struct {
  uint8_t seqNumber;
  uint16_t srcAddr;
//  uint16_t srcPanId;
  int8_t length;
  int8_t max_length;
  uint8_t *pPayload;
  bool ackRequest;
  int8_t rssi;
} RF_RX_INFO;
//-------------------------------------------------------------------------------------------------------

#define rf_polling_rx_packet rf_rx_packet_nonblock
int8_t rf_rx_packet_nonblock();

/* NOT IMPLEMENTED
uint8_t rf_rx_check_sfd();
uint8_t rf_rx_check_fifop();
*/

//-------------------------------------------------------------------------------------------------------
//  void rf_rx_on(void)
//
//  DESCRIPTION:
//      Enables the CC2420 receiver and the FIFOP interrupt. When a packet is received through this
//      interrupt, it will call halRfReceivePacket(...), which must be defined by the application
//-------------------------------------------------------------------------------------------------------
void rf_rx_on(void);
void rf_polling_rx_on(void);


//-------------------------------------------------------------------------------------------------------
//  void rf_rx_off(void)
//
//  DESCRIPTION:
//      Disables the CC2420 receiver and the FIFOP interrupt.
//-------------------------------------------------------------------------------------------------------
void rf_rx_off(void);


//-------------------------------------------------------------------------------------------------------
//  SIGNAL(SIG_INTERRUPT0) - CC2420 FIFOP interrupt service routine
//
//  DESCRIPTION:
//		When a packet has been completely received, this ISR will extract the data from the RX FIFO, put
//		it into the active RF_RX_INFO structure, and call basicRfReceivePacket() (defined by the
//		application). FIFO overflow and illegally formatted packets is handled by this routine.
//
//      Note: Packets are acknowledged automatically by CC2420 through the auto-acknowledgment feature.
//-------------------------------------------------------------------------------------------------------
// SIGNAL(SIG_INTERRUPT0);


//-------------------------------------------------------------------------------------------------------
//  RF_RX_INFO* rf_rx_callback(RF_RX_INFO *pRRI)
//
//  DESCRIPTION:
//      This function is a part of the basic RF library, but must be declared by the application. Once
//		the application has turned on the receiver, using rf_rx_on(), all incoming packets will
//		be received by the FIFOP interrupt service routine. When finished, the ISR will call the
//		rf_rx_callback() function. Please note that this function must return quickly, since the
//		next received packet will overwrite the active RF_RX_INFO structure (pointed to by pRRI).
//
//  ARGUMENTS:
//		RF_RX_INFO *pRRI
//	      	The reception structure, which contains all relevant info about the received packet.
//
//  RETURN VALUE:
//     RF_RX_INFO*
//			The pointer to the next RF_RX_INFO structure to be used by the FIFOP ISR. If there is
//			only one buffer, then return pRRI.
//-------------------------------------------------------------------------------------------------------
RF_RX_INFO* rf_rx_callback(RF_RX_INFO *pRRI);




/*******************************************************************************************************
 *******************************************************************************************************
 **************************                  Initialization                   **************************
 *******************************************************************************************************
 *******************************************************************************************************/


//-------------------------------------------------------------------------------------------------------
// The RF settings structure:
typedef struct {
    RF_RX_INFO *pRxInfo;
    uint8_t txSeqNumber;
    volatile bool ackReceived;
    uint16_t panId;
    uint16_t myAddr;
    bool receiveOn;
} RF_SETTINGS;
extern volatile RF_SETTINGS rfSettings;
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//  void rf_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr)
//
//  DESCRIPTION:
//      Initializes CC2420 for radio communication via the basic RF library functions. Turns on the
//		voltage regulator, resets the CC2420, turns on the crystal oscillator, writes all necessary
//		registers and protocol addresses (for automatic address recognition). Note that the crystal
//		oscillator will remain on (forever).
//
//  ARGUMENTS:
//      RF_RX_INFO *pRRI
//          A pointer the RF_RX_INFO data structure to be used during the first packet reception.
//			The structure can be switched upon packet reception.
//      uint8_t channel
//          The RF channel to be used (11 = 2405 MHz to 26 = 2480 MHz)
//      uint16_t panId
//          The personal area network identification number
//      uint16_t myAddr
//          The 16-bit short address which is used by this node. Must together with the PAN ID form a
//			unique 32-bit identifier to avoid addressing conflicts. Normally, in a 802.15.4 network, the
//			short address will be given to associated nodes by the PAN coordinator.
//-------------------------------------------------------------------------------------------------------
void rf_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr);
void rf_set_rx(RF_RX_INFO *pRRI, uint8_t channel);

nrk_sig_t nrk_rx_signal_get();

#endif
