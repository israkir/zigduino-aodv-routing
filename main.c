#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/sleep.h>
#include <hal.h>
#include <aodv.h>
#include <nrk_error.h>
#include <nrk_driver_list.h>
#include <nrk_driver.h>
#include <adc_driver.h>
#include "my_basic_rf.h"

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

void nrk_create_taskset ();

uint8_t rxmsg[100];
uint8_t msg[32];

RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;

uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

nrk_sig_t signal_send_packet;

//TX Flags
AODV_RREQ_INFO* RREQ = NULL;
AODV_RREP_INFO* RACK = NULL;
AODV_RREP_INFO* RREP = NULL;
AODV_RERR_INFO* RERR = NULL;

uint8_t rf_ok;
uint8_t broadcast_id = 0;

void init_srand_seed() {
  int light_val;
  uint8_t val;
  uint8_t buf;
  int8_t fd;
  fd = nrk_open(ADC_DEV_MANAGER, READ);
  if (fd==NRK_ERROR) 
    nrk_kprintf(PSTR("Failed to open adc driver\r\n"));
  val = nrk_set_status(fd, ADC_CHAN, 0);
  val = nrk_read(fd, &buf, 1);
  light_val = buf;
  nrk_close(fd);
  srand(light_val);
}

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);
  
  // init a unique id for this node
  init_srand_seed();
  node_addr = rand() % 1000;

  nrk_init ();
  nrk_time_set (0, 0);

  rf_ok = 0;
  nrk_create_taskset ();
  nrk_start ();
  
  return 0;
}

void rx_task ()
{
  uint8_t i, len;
  int8_t n;
  uint8_t *local_rx_buf;
  uint8_t type;
  uint8_t next_hop;
  AODV_MSG_INFO aodvmsg;
  AODV_RREQ_INFO aodvrreq;
  AODV_RREP_INFO aodvrrep;
  AODV_RREP_INFO aodvrack;
  AODV_RERR_INFO aodvrerr;

  printf ("rx_task PID=%d\r\n", nrk_get_pid ());

  set_routing_table();
  // Init basic rf 
  rfRxInfo.pPayload = rx_buf;
  rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
  rfRxInfo.ackRequest = 1;
  nrk_int_enable();
  rf_init (&rfRxInfo, 26, 0xffff, node_addr);

  rf_polling_rx_on();
  
  printf( "Waiting for packet...\r\n" );
  
  nrk_sig_t rx_signal;
  // Get the signal for radio RX
  rx_signal = nrk_rx_signal_get();
  // Register task to wait on signal
  nrk_signal_register(rx_signal);	
  printf("rx before while loop\r\n");
  // set rf ready flag
  rf_ok = 1;

  while (1) {
    // Wait until an RX packet is received
    while ((n = rf_polling_rx_packet ()) == 0){
    }
    
    nrk_led_set (RFRX_LED);
    if(n == NRK_OK){
      printf("****** packet received length = %d ", rfRxInfo.length);

      printf("SEQNUM: %d [", rfRxInfo.seqNumber);
      //printf("SEQNUM: %d  SRCADDR: 0x%x  SNR: %d\r\n[",
      //        rfRxInfo.seqNumber, rfRxInfo.srcAddr, rfRxInfo.rssi);

      for(i=0; i<rfRxInfo.length; i++ )
        printf( "%c", rfRxInfo.pPayload[i]);
      printf( "] ******\r\n" );
    }else if(n == NRK_ERROR){
      printf( "CRC failed!\r\n" );
    }
    
    local_rx_buf = rfRxInfo.pPayload;
    len = rfRxInfo.length;
    
    // Get the aodv msg type 
    type = get_msg_type(local_rx_buf);
    if (type == 0){ //normal msg
      unpack_aodv_msg (local_rx_buf, &aodvmsg, rxmsg);
      printf("\r\ntype = %d, src = %d, nexthop = %d, dest = %d, msg_len = %d, msg = %s\r\n", 
        aodvmsg.type, aodvmsg.src, aodvmsg.next_hop, aodvmsg.dest, aodvmsg.msg_len, aodvmsg.msg);
      //pass_aodv_msg(uint8_t* rx_buf, AODV_MSG_INFO* aodvmsg);

      if(aodvmsg.next_hop == node_addr){
        // this AODV msg is for this node, so process it!
        if(aodvmsg.dest == node_addr){
          // this node is destination, so print received packet
          printf("!!!!got msg!!!!\r\n");
          printf("\r\ntype = %d, src = %d, nexthop = %d, dest = %d, msg_len = %d, msg = %s\r\n", 
            aodvmsg.type, aodvmsg.src, aodvmsg.next_hop, aodvmsg.dest, aodvmsg.msg_len, aodvmsg.msg);
        }else{
          // this node is not destination, so send it to neighbor
          if((next_hop = find_next_hop(aodvmsg.dest)) != 0){
            printf("sendmsg to %d\r\n", next_hop);
            repack_forward_msg(local_rx_buf, aodvmsg, next_hop);
            send_packet(local_rx_buf, len);
          } else {
            // routing information is not found in the routing table, so issue a new RREQ!
            if (aodvmsg.src == node_addr) {
              broadcast_id++;
              // construct RREQ message
              aodvrreq.type = 1;
              aodvrreq.broadcast_id = broadcast_id;
              aodvrreq.src = aodvmsg.src;
              aodvrreq.src_seq_num = 1; 
              aodvrreq.dest = aodvmsg.dest;
              aodvrreq.dest_seq_num = 0; // 0 means unknown.
              aodvrreq.hop_count = 1;
              // set flag for tx_task, so tx_task can broadcast!
              RREQ = &aodvrreq;
            }
            /*
            else {
              // TODO: check about this case
            }
            */
          }
        }
      }
    } else if(type == 1) { // RREQ
        unpack_aodv_rreq(local_rx_buf, &aodvrreq);
        // if received rreq is valid, then process it!
        // valid: this node did not received a RREQ with the same broadcast_id & source addr
        if (check_rreq_is_valid(&aodvrreq) != -1) {
          // create inverse routing entry
          add_routing_entry(aodvrreq.src, rfRxInfo.srcAddr, aodvrreq.src_seq_num, aodvrreq.hop_count, rfRxInfo.rssi);         
          // this node neighbor of destination, so RREP!
          if ((aodvrreq.dest == find_next_hop(aodvrreq.dest)) || aodvrreq.dest == node_addr) {
            if (dest_seq_num < aodvrreq.dest_seq_num) {
              aodvrrep.type = 2;
              aodvrrep.src = aodvrreq.src;
              aodvrrep.dest = aodvrreq.dest;
              aodvrrep.dest_seq_num = dest_seq_num;
              aodvrrep.hop_count = 1;
              RREQ = NULL;
              RREP = &aodvrrep;
            } else {
              // this node is not neighbor to destination, so propagate RREQ!
              RREP = NULL;
              aodvrreq.hop_count += 1;
              RREQ = &aodvrreq;
            }
          } else {
            // this node is not destination, so propagate RREQ!
            RREP = NULL;        
            aodvrreq.hop_count += 1;
            RREQ = &aodvrreq;
          }
        }
    } else if(type == 2) { // RREP
      unpack_aodv_rrep (local_rx_buf, &aodvrrep);
      printf("type = %d, src = %d, dest = %d, dest_seq_num = %d, hop_count = %d, lifespan = %d\r\n", 
        aodvrrep.type, aodvrrep.src, aodvrrep.dest, aodvrrep.dest_seq_num, aodvrrep.hop_count, aodvrrep.lifespan);

      if (dest_seq_num < aodvrrep.dest_seq_num) {
        // update the destination sequence number
        dest_seq_num = aodvrrep.dest_seq_num;

        // TODO: this is for timeout of the reverse route entries
        // renew routing table entries to source and destination
        // renew_routing_entry(aodvrrep.src);
        // renew_routing_entry(aodvrrep.dest);
        RREQ = NULL;
        RREP = &aodvrrep;
      } else {
        RREQ = NULL;
        RREP = NULL;
      }
    } else if(type == 3) { //RERR
	
		unpack_aodv_rerr (local_rx_buf, &aodvrerr);
		printf("type = %d, dest = %d\r\n", aodvmsg.type, aodvmsg.dest);
	  
		// delete route that contains broken link from the routing table
		remove_routing_entry(); //Remove function not finished yet
	  
    } else if(type == 4) { // RACK (special RREP)
      /*
      unpack_aodv_rreq (local_rx_buf, &aodvrreq);
      printf("\r\ntype = %d, broadcast_id = %d, src = %d, src_seq_num = %d, dest = %d, dest_seq_num = %d, hop_count = %d\r\n", 
        aodvrreq.type, aodvrreq.broadcast_id, aodvrreq.src, aodvrreq.src_seq_num, aodvrreq.dest, aodvrreq.dest_seq_num, aodvrreq.hop_count);

      //Update routing table
      //The RACK should always be one hop
      if (find_next_hop(aodvrack.src) == aodvrack.src) {
        update_routing_entry(aodvrreq.dest, aodvrreq.dest, aodvrreq.broadcast_id, aodvrreq.hop_count, rfRxInfo.rssi);
      }
      else {
        //For first time discovery
        add_routing_entry(aodvrreq.dest, aodvrreq.dest, aodvrreq.broadcast_id, aodvrreq.hop_count, rfRxInfo.rssi);
      } */
    } else {
      nrk_kprintf( PSTR("unknown type\r\n"));
    }

    //printf ("Got RX packet len=%d RSSI=%d [", len, rssi);
    //for (i = 0; i < len; i++)
    //	printf ("%c", rx_buf[i]);
    //printf ("]\r\n");
    nrk_led_clr (RFRX_LED);
    nrk_event_wait(SIG(rx_signal));
  }
}


void tx_task ()
{
  AODV_MSG_INFO aodvmsg;
  AODV_RREQ_INFO aodvrreq;
  AODV_RREP_INFO aodvrrep;
  AODV_RREP_INFO aodvrack;

  signal_send_packet = nrk_rx_signal_get();
  nrk_signal_register(signal_send_packet);	

  while(!rf_ok){
    nrk_wait_until_next_period();
  }
  printf ("tx_task PID=%d\r\n", nrk_get_pid ());

  while (1) {

    nrk_led_set(RFTX_LED);

    nrk_event_wait(SIG(signal_send_packet));

    // RACK
    /*
    if () {
      aodvrreq.type = 4;
      aodvrreq.broadcast_id = 0;
      aodvrreq.dest = node_addr;
      aodvrreq.lifespan = 1;
      aodvrreq.hop_count = 1;

      pack_aodv_rreq(tx_buf, aodvrreq);
      printf("txpacket type = %d, broadcast_id = %d, dest = %d, lifespan = %d, hop_count = %d\r\n", 
        tx_buf[0], tx_buf[1], tx_buf[3], tx_buf[4], tx_buf[5]);

      send_packet(tx_buf, sizeof(tx_buf));
    }
    */

    if (RREP) {
      if (strcmp(WHOAMI, "destination") == 0) {
        node_seq_num++;
      }
      aodvrrep = *RREP;
      pack_aodv_rrep(tx_buf, aodvrrep);
      send_rrep(tx_buf, sizeof(tx_buf), find_next_hop(aodvrrep.src));
    }

    if (RREQ) {
      if (strcmp(WHOAMI, "source") == 0) {
        node_seq_num++;
      }
      aodvrreq = *RREQ;
      pack_aodv_rreq(tx_buf, aodvrreq);
      broadcast_rreq(tx_buf, sizeof(tx_buf));
    }
  

    /*
    if () {
      aodvmsg.dest = node_addr;
      aodvmsg.type = 0;
      aodvmsg.src = node_addr;
      aodvmsg.length = strlen(msg)+1;
      aodvmsg.msg = msg;

      if((aodvmsg.next_hop = find_next_hop(aodvmsg.dest)) != 0){

        pack_aodv_msg(tx_buf, aodvmsg);
        printf("txpacket type = %d, src = %d, next_hop = %d, dest = %d\r\n", 
          tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
        // Build the TX packet
        rfTxInfo.pPayload = tx_buf;
        rfTxInfo.length = aodvmsg.length+5;
        rfTxInfo.destAddr = tx_buf[2];
        rfTxInfo.cca = 0;
        rfTxInfo.ackRequest = 1;

        //printf( "Sending\r\n" );
        if(rf_tx_packet(&rfTxInfo) != 1){
          printf("@@@ RF_TX ERROR @@@\r\n");
        }else{
          printf("--- RF_TX ACK!! ---\r\n");
        }
        //nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
      }
    }
    */

    nrk_led_clr(RFTX_LED);
    nrk_wait_until_next_period();
  }
}

void nrk_create_taskset ()
{
  RX_TASK.task = rx_task;
  nrk_task_set_stk( &RX_TASK, rx_task_stack, NRK_APP_STACKSIZE);
  RX_TASK.prio = 2;
  RX_TASK.FirstActivation = TRUE;
  RX_TASK.Type = BASIC_TASK;
  RX_TASK.SchType = PREEMPTIVE;
  RX_TASK.period.secs = 1;
  RX_TASK.period.nano_secs = 0;
  RX_TASK.cpu_reserve.secs = 1;
  RX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  RX_TASK.offset.secs = 0;
  RX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&RX_TASK);

  TX_TASK.task = tx_task;
  nrk_task_set_stk( &TX_TASK, tx_task_stack, NRK_APP_STACKSIZE);
  TX_TASK.prio = 2;
  TX_TASK.FirstActivation = TRUE;
  TX_TASK.Type = BASIC_TASK;
  TX_TASK.SchType = PREEMPTIVE;
  TX_TASK.period.secs = 0;
  TX_TASK.period.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.cpu_reserve.secs = 1;
  TX_TASK.cpu_reserve.nano_secs = 500 * NANOS_PER_MS;
  TX_TASK.offset.secs = 0;
  TX_TASK.offset.nano_secs = 0;
  nrk_activate_task (&TX_TASK);

  printf ("Create done\r\n");
}
