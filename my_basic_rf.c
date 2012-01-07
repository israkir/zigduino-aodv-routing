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
*  Maxim Buevich
*  Anthony Rowe
*******************************************************************************/

#include <include.h>
#include <ulib.h>
#include <nrk.h>
#include <nrk_events.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_cpu.h>
#include "my_basic_rf.h"

#define OSC_STARTUP_DELAY	1000

//#define RADIO_VERBOSE
#ifdef RADIO_VERBOSE
  #define vprintf(...)		printf(__VA_ARGS__)
#else
  #define vprintf(...) 	
#endif


typedef struct ieee_mac_fcf{
  uint8_t frame_type: 		3;
  uint8_t sec_en: 				1;
  uint8_t frame_pending: 	1;
  uint8_t ack_request: 		1;
  uint8_t intra_pan: 			1;

  uint8_t res:						3;	

  uint8_t dest_addr_mode:	2;
  uint8_t frame_version: 	2;
  uint8_t src_addr_mode:	2;
} ieee_mac_fcf_t;

typedef struct ieee_mac_frame_header{	
  ieee_mac_fcf_t fcf;
  uint8_t seq_num;

  uint16_t dest_pan_id;
  uint16_t dest_addr;
  //uint16_t src_pan_id;
  uint16_t src_addr;
  /*uint16_t sec_header; */
} ieee_mac_frame_header_t;



static void rf_cmd(uint8_t cmd);

static nrk_sig_t rx_signal;

nrk_sem_t *radio_sem;
//uint8_t auto_ack_enable;
//uint8_t security_enable;
//uint8_t last_pkt_encrypted;
uint16_t mdmctrl0;
uint8_t tx_ctr[6];
uint8_t rx_ctr[4];

volatile RF_SETTINGS rfSettings;
uint8_t rf_ready;
volatile uint8_t rx_ready;
volatile uint8_t tx_done;

nrk_time_t curr_t, target_t, dummy_t;

void rf_power_down()
{
  uint8_t status;

  while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
    continue;

  /* For some reason comparing to SLEEP doesn't work, but 0 does */
  status = (TRX_STATUS & 0x1F);
  if((status == 0) || (status == 0xF))
    return;
  /* Disable TRX if it is enabled */
  if((TRX_STATUS & 0x1F) != TRX_OFF){
    rf_cmd(TRX_OFF);
    do{
      status = (TRX_STATUS & 0x1F);
    }while(status != TRX_OFF);
  }

  TRXPR |= (1 << SLPTR);
  do{
    status = (TRX_STATUS & 0x1F);
  }while((status != 0) && (status != 0xF));
}

void rf_power_up()
{
  uint8_t status;

  while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
    continue;
  /* For some reason comparing to SLEEP doesn't work, but 0 does */
  status = (TRX_STATUS & 0x1F);
  if((status != 0) && (status != 0xF))
    return;

  /* Wake up */
  TRXPR &= ~(1 << SLPTR);
  while((TRX_STATUS & 0x1F) != TRX_OFF)
    continue;
}


/* Safely change the radio state */
static void rf_cmd(uint8_t cmd)
{
  while((TRX_STATUS & 0x1F) == STATE_TRANSITION_IN_PROGRESS)
    continue;
  TRX_STATE = cmd;
}


void rf_tx_power(uint8_t pwr)
{
  PHY_TX_PWR &= 0xF0;
  PHY_TX_PWR |= (pwr & 0xF);
}

void rf_addr_decode_enable()
{
  XAH_CTRL_1 &= ~(1 << AACK_PROM_MODE);
}


void rf_addr_decode_disable()
{
  XAH_CTRL_1 |= (1 << AACK_PROM_MODE);
}


void rf_auto_ack_enable()
{
  CSMA_SEED_1 &= ~(1 << AACK_DIS_ACK);
}

void rf_auto_ack_disable()
{
  CSMA_SEED_1 |= (1 << AACK_DIS_ACK);
}


void rf_addr_decode_set_my_mac(uint16_t my_mac)
{
  /* Set short MAC address */
  SHORT_ADDR_0 = (my_mac & 0xFF); 
  SHORT_ADDR_1 = (my_mac >> 8);
  rfSettings.myAddr = my_mac;
}


void rf_set_rx(RF_RX_INFO *pRRI, uint8_t channel )
{
  rfSettings.pRxInfo = pRRI;
  PHY_CC_CCA &= ~(0x1F);
  PHY_CC_CCA |= (channel << CHANNEL0);
}



void rf_init(RF_RX_INFO *pRRI, uint8_t channel, uint16_t panId, uint16_t myAddr)
{ 
/*
   uint8_t n;
   int8_t v;

#ifdef RADIO_PRIORITY_CEILING
    radio_sem = nrk_sem_create(1,RADIO_PRIORITY_CEILING);
    if (radio_sem == NULL)
      nrk_kernel_error_add (NRK_SEMAPHORE_CREATE_ERROR, nrk_get_pid ());

  v = nrk_sem_pend (radio_sem);
  if (v == NRK_ERROR) {
    nrk_kprintf (PSTR ("CC2420 ERROR:  Access to semaphore failed\r\n"));
  }
#endif
  

#ifdef RADIO_PRIORITY_CEILING
  v = nrk_sem_post (radio_sem);
  if (v == NRK_ERROR) {
    nrk_kprintf (PSTR ("CC2420 ERROR:  Release of semaphore failed\r\n"));
    _nrk_errno_set (2);
  }
#endif

*/


  /* Turn on auto crc calculation */
  TRX_CTRL_1 = (1 << TX_AUTO_CRC_ON);
  /* Set PA buffer lead time to 6 us and TX power to 3.0 dBm (maximum) */
  PHY_TX_PWR = (1 << PA_BUF_LT1) | (1 << PA_BUF_LT0) | (0 << TX_PWR0);
  /* CCA Mode and Channel selection */
  PHY_CC_CCA = (0 << CCA_MODE1) | (1 << CCA_MODE0) | (channel << CHANNEL0);
  /* Set CCA energy threshold */
  CCA_THRES = 0xC5;
  /* Start of frame delimiter */
  SFD_VALUE = 0xA7;
  /* Dynamic buffer protection on and data rate is 250 kb/s */
  TRX_CTRL_2 = (1 << RX_SAFE_MODE) | (0 << OQPSK_DATA_RATE1) | (0 << OQPSK_DATA_RATE0);
  
  /* Set short MAC address */
  SHORT_ADDR_0 = (myAddr & 0xFF); SHORT_ADDR_1 = (myAddr >> 8);
  /* Set PAN ID */
  PAN_ID_0 = (panId & 0xFF); PAN_ID_1 = (panId >> 8);
  
  /* 2-bit random value generated by radio hardware */
  #define RADIO_RAND ((PHY_RSSI >> RND_VALUE0) & 0x3)
  /* Set random csma seed */
  CSMA_SEED_0 = (RADIO_RAND << 6) | (RADIO_RAND << 4) 
      | (RADIO_RAND << 2) | (RADIO_RAND << 0);
  /* Will ACK received frames with version numbers of 0 or 1 */
  CSMA_SEED_1 = (0 << AACK_FVN_MODE1) | (1 << AACK_FVN_MODE0) 
      | (RADIO_RAND << CSMA_SEED_11) | (RADIO_RAND << CSMA_SEED_10);

  /* don't re-transmit frames or perform cca multiple times, slotted op is off */
  XAH_CTRL_0 = (0 << MAX_FRAME_RETRIES0) | (0 << MAX_CSMA_RETRIES0)
      | (0 << SLOTTED_OPERATION);
  /* Enable radio interrupts */
  IRQ_MASK = (1 << AWAKE_EN) | (1 << TX_END_EN) | (1 << AMI_EN) | (1 << CCA_ED_DONE_EN)
      | (1 << RX_END_EN) | (1 << RX_START_EN) | (1 << PLL_UNLOCK_EN) | (1 << PLL_LOCK_EN);

  /* Initialize settings struct */
  rfSettings.pRxInfo = pRRI;
  rfSettings.txSeqNumber = 0;
  rfSettings.ackReceived = 0;
  rfSettings.panId = panId;
  rfSettings.myAddr = myAddr;
  rfSettings.receiveOn = 0;

  rf_ready = 1;
  rx_ready = 0;
  tx_done = 0;
  rx_signal=nrk_signal_create();
  if(rx_signal==NRK_ERROR) nrk_error_add(NRK_SIGNAL_CREATE_ERROR);

} // rf_init() 
nrk_sig_t nrk_rx_signal_get()
{
  if(rx_signal==NRK_ERROR) nrk_error_add(NRK_SIGNAL_CREATE_ERROR);
  return rx_signal;
}


//-------------------------------------------------------------------------------------------------------
//  void rf_rx_on(void)
//
//  DESCRIPTION:
//      Enables the CC2420 receiver and the FIFOP interrupt. When a packet is received through this
//      interrupt, it will call rf_rx_callback(...), which must be defined by the application
//-------------------------------------------------------------------------------------------------------
void rf_rx_on(void) 
{
/*
#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend (radio_sem);
#endif
  rfSettings.receiveOn = TRUE;

#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
#endif
*/
  rf_cmd(RX_AACK_ON);
}

void rf_polling_rx_on(void)
{
/*#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend (radio_sem);
#endif
  rfSettings.receiveOn = TRUE;


#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
#endif
*/
  rf_cmd(RX_AACK_ON);
}


//-------------------------------------------------------------------------------------------------------
//  void rf_rx_off(void)
//
//  DESCRIPTION:
//      Disables the CC2420 receiver and the FIFOP interrupt.
//-------------------------------------------------------------------------------------------------------
void rf_rx_off(void)
{
/*
#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend (radio_sem);
#endif
  // XXX
  //SET_VREG_INACTIVE();	
  rfSettings.receiveOn = FALSE;

#ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
#endif
  //	DISABLE_FIFOP_INT();
*/
  rf_cmd(TRX_OFF);
  rx_ready = 0;
}



//-------------------------------------------------------------------------------------------------------
//  BYTE rf_tx_packet(RF_TX_INFO *pRTI)
//
//  DESCRIPTION:
//		Transmits a packet using the IEEE 802.15.4 MAC data packet format with short addresses. CCA is
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
//		uint8_t
//			Successful transmission (acknowledgment received)
//-------------------------------------------------------------------------------------------------------

uint8_t rf_tx_packet(RF_TX_INFO *pRTI){
  return rf_tx_packet_repeat(pRTI, 0);
}

uint8_t rf_tx_packet_repeat(RF_TX_INFO *pRTI, uint16_t ms)
{
  /*
  #ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend(radio_sem);
  #endif

  #ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
  #endif
  //return success;
  */
  

  uint8_t trx_status, trx_error, *data_start, *frame_start = &TRXFBST;
  uint16_t i;

  if(!rf_ready)
    return NRK_ERROR;

  ieee_mac_frame_header_t *machead = frame_start + 1;
  ieee_mac_fcf_t fcf;

  /* TODO: Setting FCF bits is probably slow. Optimize later. */
  fcf.frame_type = 1;
  fcf.sec_en = 0;
  fcf.frame_pending = 0;
  fcf.ack_request = pRTI->ackRequest;
  fcf.intra_pan = 1;
  fcf.res = 0;
  fcf.dest_addr_mode = 2;
  fcf.frame_version = 0;
  fcf.src_addr_mode = 2;

  /* Build the rest of the MAC header */
  rfSettings.txSeqNumber++;
  machead->fcf = fcf;
  machead->seq_num = rfSettings.txSeqNumber;
  machead->dest_pan_id = (PAN_ID_1 << 8) | PAN_ID_0;
  machead->dest_addr = pRTI->destAddr;
  //machead->src_pan_id = (PAN_ID_1 << 8) | PAN_ID_0;
  machead->src_addr = (SHORT_ADDR_1 << 8) | SHORT_ADDR_0;

  /* Copy data payload into packet */
  data_start = frame_start + sizeof(ieee_mac_frame_header_t) + 1;
  memcpy(data_start, pRTI->pPayload, pRTI->length);
  /* Set the size of the packet */
  *frame_start = sizeof(ieee_mac_frame_header_t) + pRTI->length + 2;
  
  vprintf("packet length: %d bytes\r\n", *frame_start);

  /* Wait for radio to be in a ready state */
  do{
    trx_status = (TRX_STATUS & 0x1F);
  }while((trx_status == BUSY_TX) || (trx_status == BUSY_RX)
      || (trx_status == BUSY_RX_AACK) || (trx_status == BUSY_TX_ARET)
      || (trx_status == STATE_TRANSITION_IN_PROGRESS));
  
  /* Return error if radio not in a tx-ready state */
  if((trx_status != TRX_OFF) && (trx_status != RX_ON) 
      && (trx_status != RX_AACK_ON) && (trx_status != PLL_ON)){
    return NRK_ERROR;
  }

  rf_cmd(RX_AACK_ON);

  /* Perform CCA if requested */
  if(pRTI->cca){
    PHY_CC_CCA |= (1 << CCA_REQUEST);
    while(!(TRX_STATUS & (1 << CCA_DONE)))
      continue;
    if(!(TRX_STATUS & (1 << CCA_STATUS)))
      return NRK_ERROR;
  }

  rf_cmd(PLL_ON);
  if(pRTI->ackRequest)
    rf_cmd(TX_ARET_ON);
  
  if(ms != 0){
    nrk_time_get(&curr_t);
    target_t.secs = curr_t.secs;
    target_t.nano_secs = curr_t.nano_secs + (ms * NANOS_PER_MS);
    nrk_time_compact_nanos(&target_t);
  }
  
  do{
    tx_done = 0;
    /* Send the packet. 0x2 is equivalent to TX_START */
    rf_cmd(0x2);

    /* Return an error if no ACK received */
    for(i=0; (i<65000) && !tx_done; i++)
      continue;
    if(ms == 0)
      break;
    nrk_time_get(&curr_t);
  }while(nrk_time_sub(&dummy_t, target_t, curr_t) != NRK_ERROR);

  trx_error = ((pRTI->ackRequest && 
      (((TRX_STATE >> TRAC_STATUS0) & 0x7) != 0))
      || (i == 65000)) ? NRK_ERROR : NRK_OK;
  rf_cmd(trx_status);

  return trx_error;
}


/* Returns 1 if the channel is clear
 * Returns 0 if the channel is being used
 */
int8_t rf_cca_check()
{
  uint8_t trx_status, cca_value;

  if(!rf_ready)
    return NRK_ERROR;

  /* Wait for radio to be in a ready state */
  do{
    trx_status = (TRX_STATUS & 0x1F);
  }while((trx_status == BUSY_TX) || (trx_status == BUSY_RX)
      || (trx_status == BUSY_RX_AACK) || (trx_status == BUSY_TX_ARET)
      || (trx_status == STATE_TRANSITION_IN_PROGRESS)); 

  /* Return error if radio not in a tx-ready state */
  if((trx_status != TRX_OFF) && (trx_status != RX_ON) 
      && (trx_status != RX_AACK_ON))
    return NRK_ERROR;
  
  rf_cmd(RX_AACK_ON);

  PHY_CC_CCA |= (1 << CCA_REQUEST);
  while(!(TRX_STATUS & (1 << CCA_DONE)))
    continue;
  cca_value = (TRX_STATUS & (1 << CCA_STATUS)) ? 1 : 0;
  rf_cmd(trx_status);

  return cca_value;
}



int8_t rf_rx_packet_nonblock()
{
  /*
  #ifdef RADIO_PRIORITY_CEILING
  nrk_sem_pend(radio_sem);
  #endif
  
  #ifdef RADIO_PRIORITY_CEILING
  nrk_sem_post(radio_sem);
  #endif
  */
  
  uint8_t *frame_start = &TRXFBST;

  if(!rf_ready)
    return NRK_ERROR;

  if(!rx_ready)
    return 0;
  else if((TST_RX_LENGTH - 2) > rfSettings.pRxInfo->max_length)
    return NRK_ERROR;

  ieee_mac_frame_header_t *machead = frame_start;

  rfSettings.pRxInfo->seqNumber = machead->seq_num;
  rfSettings.pRxInfo->srcAddr = machead->src_addr;
  rfSettings.pRxInfo->length = TST_RX_LENGTH - sizeof(ieee_mac_frame_header_t) - 2;

  if((rfSettings.pRxInfo->length > rfSettings.pRxInfo->max_length)
      || (rfSettings.pRxInfo->length < 0)){
    rx_ready = 0;
    TRX_CTRL_2 &= ~(1 << RX_SAFE_MODE);
    TRX_CTRL_2 |= (1 << RX_SAFE_MODE);
    return NRK_ERROR;
  }

  memcpy(rfSettings.pRxInfo->pPayload, frame_start 
      + sizeof(ieee_mac_frame_header_t), rfSettings.pRxInfo->length);
  
  /* I am assuming that ackRequest is supposed to
   * be set, not read, by rf_basic */
  rfSettings.pRxInfo->ackRequest = machead->fcf.ack_request;
  //rfSettings.pRxInfo->rssi = *(frame_start + TST_RX_LENGTH);
  rfSettings.pRxInfo->rssi = PHY_ED_LEVEL;

  /* Reset frame buffer protection */
  rx_ready = 0;
  TRX_CTRL_2 &= ~(1 << RX_SAFE_MODE);
  TRX_CTRL_2 |= (1 << RX_SAFE_MODE);

  return NRK_OK;
}


SIGNAL(TRX24_RX_END_vect)
{	
  uint8_t i, *byte_ptr = &TRXFBST;

  /* Verbose mode print block */
  vprintf("RX_END IRQ!\r\n");	
  for(i=0; i<TST_RX_LENGTH; i++){
    vprintf("0x%x ", byte_ptr[i]);
    if(((i+1) % 16) == 0)
      vprintf("\r\n");
  }
  vprintf("\r\n");


  // Get the signal for UART RX
  // Register task to wait on signal

  if((PHY_RSSI >> RX_CRC_VALID) & 0x1)
    rx_ready = 1;
  IRQ_STATUS = (1 << RX_END);
  nrk_event_signal(rx_signal);

  return;
}


/* These interrupt handlers are useful for finding
 * out the exact order of events during a transmission */

SIGNAL(TRX24_AWAKE_vect)
{
  vprintf("RADIO AWAKE IRQ!\r\n");
  IRQ_STATUS = (1 << AWAKE);

  return;
}

SIGNAL(TRX24_TX_END_vect)
{
  vprintf("TX_END IRQ!\r\n");
  tx_done = 1;
  IRQ_STATUS = (1 << TX_END);

  return;
}

SIGNAL(TRX24_XAH_AMI_vect)
{
  vprintf("AMI IRQ!\r\n");
  IRQ_STATUS = (1 << AMI);

  return;
}

SIGNAL(TRX24_CCA_ED_DONE_vect)
{
  vprintf("CCA_ED_DONE IRQ!\r\n");
  IRQ_STATUS = (1 << CCA_ED_DONE);

  return;
}

SIGNAL(TRX24_RX_START_vect)
{
  vprintf("RX_START IRQ!\r\n");
  IRQ_STATUS = (1 << RX_START);

  return;
}

SIGNAL(TRX24_PLL_UNLOCK_vect)
{
  vprintf("PLL_UNLOCK IRQ!\r\n");
  IRQ_STATUS = (1 << PLL_UNLOCK);

  return;
}

SIGNAL(TRX24_PLL_LOCK_vect)
{
  vprintf("PLL_LOCK IRQ!\r\n");
  IRQ_STATUS = (1 << PLL_LOCK);

  return;
}



void rf_set_cca_thresh(int8_t t)
{
  CCA_THRES &= 0xF0;
  CCA_THRES |= (t & 0xF);
  return;
}


// Returns 1 if the last packet was encrypted, 0 otherwise
uint8_t rf_security_last_pkt_status()
{
  //return last_pkt_encrypted;
  return NRK_ERROR;
}


void rf_security_set_ctr_counter(uint8_t *counter)
{
  return;
}


void rf_security_set_key(uint8_t *key)
{
  return;
}



void rf_security_disable()
{
  return;
}



/**********************************************************
 ******************* NOT IMPLEMENTED **********************
 **********************************************************/


uint8_t rf_tx_tdma_packet(RF_TX_INFO *pRTI, uint16_t slot_start_time, uint16_t tx_guard_time) {
  //    return success;
  return NRK_ERROR;
}


nrk_sem_t* rf_get_sem()
{
  return radio_sem;
}


int8_t rf_rx_packet()
{
  /*
     int8_t tmp;
     if(rx_ready>0) { tmp=rx_ready; rx_ready=0; return tmp;}
   */
  return 0;
}


inline void rf_flush_rx_fifo()
{
}

uint8_t rf_busy()
{
  //return SFD_IS_1;
  return 1;
}

/* Implement */
uint8_t rf_rx_check_fifop()
{
  //return FIFOP_IS_1;
  return 1;
}


uint8_t rf_rx_check_sfd()
{
  //return SFD_IS_1;
  return 1;
}



/**********************************************************
 * start sending a carrier pulse
 * assumes wdrf_radio_test_mode() was called before doing this
 */
void rf_carrier_on()
{
  /*
#ifdef RADIO_PRIORITY_CEILING
nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
nrk_sem_post(radio_sem);
#endif
   */
}



/**********************************************************
 * stop sending a carrier pulse; set the radio to idle state
 */
void rf_carrier_off()
{
  /*
#ifdef RADIO_PRIORITY_CEILING
nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
nrk_sem_post(radio_sem);
#endif
   */
}



void rf_test_mode()
{
  /*
#ifdef RADIO_PRIORITY_CEILING
nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
nrk_sem_post(radio_sem);
#endif
   */
}


/**********************************************************
 * set the radio into "normal" mode (buffered TXFIFO) and go into (data) receive */
void rf_data_mode()
{
  /*
#ifdef RADIO_PRIORITY_CEILING
nrk_sem_pend(radio_sem);
#endif

#ifdef RADIO_PRIORITY_CEILING
nrk_sem_post(radio_sem);
#endif
   */
}





/**********************************************************
 * Set the radio into serial unbuffered RX mode
 * RX data is received through sampling the FIFO pin, timing is done using FIFOP 
 * Use rf_rx_on() to start rcv, then wait for SFD / FIFOP. Sample during each high edge of FIFOP
 * This can be undone by using rf_data_mode()
 */
void rf_rx_set_serial()
{
}

/**********************************************************
 * Put the radio in serial TX mode, where data is sampled from the FIFO
 * pin to send after SFD, and timing is done using FIFOP
 * use rf_carrier_on() to start, set FIFO to first bit, then wait for it
 * to go up and down, then set next bit etc.
 * NOTE: You must set the FIFO pin to output mode in order to do this!
 * This can be undone by calling rf_data_mode()
 */
void rf_tx_set_serial()
{
}

/**********************************************************
 * Specifies the number of symbols to be part of preamble
 * arg is equal to number of bytes - 1.
 * (3 bytes is 802.15.4 compliant, so length arg would be 2)
 * Length arg supports values 0 to 15. See the datasheet of course for more details
 */
void rf_set_preamble_length(uint8_t length)
{
}


void rf_set_cca_mode(uint8_t mode)
{
}

