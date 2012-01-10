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

#define SRC_ADDR 5
#define DEST_ADDR 6

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

nrk_task_type SERIAL_TASK;
NRK_STK serial_task_stack[NRK_APP_STACKSIZE];
void serial_task(void);

void nrk_create_taskset ();

uint8_t rxmsg[100];
uint8_t msg[32];
uint8_t is_broadcasting = 0;

RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;

uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

//TX Flags
AODV_RREQ_INFO* RREQ = NULL;
AODV_RREP_INFO* RACK = NULL;
AODV_RREP_INFO* RREP = NULL;
AODV_RERR_INFO* RERR = NULL;
AODV_MSG_INFO* RMSG = NULL;

uint8_t rf_ok;
uint8_t broadcast_id = 0;

void init_srand_seed() {
  int light_val;
  uint8_t val;
  uint8_t buf;
  int8_t fd;
  fd = nrk_open(ADC_DEV_MANAGER, READ);
  if (fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open adc driver\r\n"));
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

  if (WHOAMI == "source") {
    node_addr = SRC_ADDR;
  } else if (WHOAMI == "destination") {
    node_addr = DEST_ADDR;
  } else {
    init_srand_seed();
    node_addr = (rand() % 1000) + 7;
  }
  
  printf("[DEBUG-main] My addr is: %d\r\n", node_addr);

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

  printf ("[DEBUG-RX] rx_task PID=%d\r\n", nrk_get_pid ());


  // set_routing_table();
  // Init basic rf 
  rfRxInfo.pPayload = rx_buf;
  rfRxInfo.max_length = RF_MAX_PAYLOAD_SIZE;
  rfRxInfo.ackRequest = 1;
  nrk_int_enable();
  rf_init (&rfRxInfo, 26, 0xffff, node_addr);

  rf_polling_rx_on();
  
  nrk_sig_t rx_signal;
  // Get the signal for radio RX
  rx_signal = nrk_rx_signal_get();
  // Register task to wait on signal
  nrk_signal_register(rx_signal);	
  // set rf ready flag
  rf_ok = 1;

  while (1) {
    // Wait until an RX packet is received
    while ((n = rf_polling_rx_packet ()) == 0) {}

    if(n == NRK_OK) {
      printf("[DEBUG-DATA] ");
      for(i=0; i<rfRxInfo.length; i++ ) printf("[%d]", rfRxInfo.pPayload[i]);
      printf("\r\n");

      local_rx_buf = rfRxInfo.pPayload;
      len = rfRxInfo.length;
      
      // Get the aodv msg type 
      type = get_msg_type(local_rx_buf);

      if (type == 0) { //normal msg
        unpack_aodv_msg (local_rx_buf, &aodvmsg, rxmsg);
        printf("[RX-RMSG] type = %d, src = %d, nexthop = %d, dest = %d, msg_len = %d, msg = %s\r\n", aodvmsg.type, aodvmsg.src, aodvmsg.next_hop, aodvmsg.dest, aodvmsg.msg_len, aodvmsg.msg);

        if(aodvmsg.next_hop == node_addr) {
          // this AODV msg is for this node, so process it!
          if(aodvmsg.dest == node_addr){
            // this node is destination, so print received packet
            printf("[RX-DATA] %s\r\n", aodvmsg.msg);
          } else {
            // this node is not destination, so send it to neighbor
            if((next_hop = find_next_hop(aodvmsg.dest)) != 0){
              printf("[RX-DATA] sendmsg to %d\r\n", next_hop);
              repack_forward_msg(local_rx_buf, aodvmsg, next_hop);
              RMSG = &aodvmsg;
              /*send_packet(local_rx_buf);*/
            } else {
              // routing information is not found in the routing table, so issue a new RREQ! might not happen at all.
              if (aodvmsg.src == node_addr) {
                broadcast_id++;
                // construct RREQ message
                aodvrreq.type = 1;
                aodvrreq.broadcast_id = broadcast_id;
                aodvrreq.src = aodvmsg.src;
                aodvrreq.src_seq_num = 1; 
                aodvrreq.dest = aodvmsg.dest;
                aodvrreq.dest_seq_num = dest_seq_num;
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
        printf("[RX-RREQ] type = %d, broadcast_id = %d, src = %d, src_seq_num = %d, dest = %d, dest_seq_num = %d, hop_count = %d\r\n", aodvrreq.type, aodvrreq.broadcast_id, aodvrreq.src, aodvrreq.src_seq_num, aodvrreq.dest, aodvrreq.dest_seq_num, aodvrreq.hop_count);

        // valid: this node did not received a RREQ with greater broadcast_id & source addr
        if (check_rreq_is_valid(&aodvrreq) != -1) {
          printf("[RX-RREQ] check is valid - adding the request to routing entry\r\n");
          // create inverse routing entry
          add_routing_entry(aodvrreq.src, rfRxInfo.srcAddr, aodvrreq.src_seq_num, aodvrreq.hop_count, rfRxInfo.rssi); 
          print_routing_table();

          // this node neighbor of destination, so RREP!
          if ((aodvrreq.dest == find_next_hop(aodvrreq.dest)) || aodvrreq.dest == node_addr) {
            printf("[RX-RREQ] this node is either destination or neighbor of the destination.\r\n");
            if ((dest_seq_num < aodvrreq.dest_seq_num) || (aodvrreq.dest_seq_num == 0)) {
              printf("[RX-RREQ] constructing rrep for receieved rreq...\r\n");
              aodvrrep.type = 2;
              aodvrrep.src = aodvrreq.src;
              aodvrrep.dest = aodvrreq.dest;
              aodvrrep.dest_seq_num = ++dest_seq_num;
              aodvrrep.hop_count = 1;
              RREQ = NULL;
              RREP = &aodvrrep;
            } else {
              printf("[RX-RREQ] invalid rreq...\r\n");
              RREP = NULL;
              RREQ = NULL;
            }
          }
        }
      } else if(type == 2) { // RREP
        unpack_aodv_rrep (local_rx_buf, &aodvrrep);
        printf("[RX-RREP] type = %d, src = %d, dest = %d, dest_seq_num = %d, hop_count = %d, lifespan = %d\r\n", aodvrrep.type, aodvrrep.src, aodvrrep.dest, aodvrrep.dest_seq_num, aodvrrep.hop_count, aodvrrep.lifespan);

        if ((dest_seq_num < aodvrrep.dest_seq_num) || (aodvrrep.dest_seq_num == 0)) {
          printf("[RX-RREP] check is valid - updating destination sequence number\r\n");
          // update the destination sequence number
          dest_seq_num = aodvrrep.dest_seq_num;

          // Creating a new or replace existing routing entry from the rrep message
          printf("[RX-RREP] adding routing table entry\r\n");
          add_routing_entry(aodvrrep.dest, rfRxInfo.srcAddr, aodvrrep.dest_seq_num, aodvrrep.hop_count, rfRxInfo.rssi); 
          print_routing_table();

          // TODO: this is for timeout of the reverse route entries
          // renew routing table entries to source and destination
          // renew_routing_entry(aodvrrep.src);
          // renew_routing_entry(aodvrrep.dest);
          
          RREQ = NULL;
          RREP = &aodvrrep;
          if (node_addr == aodvrrep.src) {
            is_broadcasting = 0;
            RREP = NULL;
          }
        } else {
          printf("[RX-RREP] invalid/outdated rrep message\r\n");
          RREQ = NULL;
          RREP = NULL;
        }
      } else if(type == 3) { // RERR
        unpack_aodv_rerr (local_rx_buf, &aodvrerr);
        printf("[RX-RERR] type = %d, dest = %d\r\n", aodvmsg.type, aodvmsg.dest);
        
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
      } else nrk_kprintf( PSTR("[DEBUG] unknown type\r\n"));
    } else if(n == NRK_ERROR) printf( "[DEBUG-RX] CRC failed!\r\n" );
  }
}


void tx_task ()
{
  AODV_MSG_INFO aodvmsg;
  AODV_RREQ_INFO aodvrreq;
  AODV_RREP_INFO aodvrrep;
  AODV_RREP_INFO aodvrack;
  AODV_RERR_INFO aodvrerr;

  while(!rf_ok) {
    nrk_wait_until_next_period();
  }
  printf ("[DEBUG-TX] tx_task PID=%d\r\n", nrk_get_pid ());

  while (1) {

    /*printf("in tx_task\r\n");*/

    if (RREP) {
      printf("[TX-RREP] inside the condition.\r\n");
      if (strcmp(WHOAMI, "destination") == 0) {
        node_seq_num++;
      }
      aodvrrep = *RREP;
      pack_aodv_rrep(tx_buf, aodvrrep);
      send_rrep(tx_buf, find_next_hop(aodvrrep.src));
      RREP = NULL;
    }

    if (RMSG) {
      printf("[TX-RMSG] inside the condition.\r\n");
      aodvmsg = *RMSG;  
      if (!is_broadcasting) {
        if((aodvmsg.next_hop = find_next_hop(aodvmsg.dest)) != 0){
          pack_aodv_msg(tx_buf, aodvmsg);
          printf("[TX-RMSG] txpacket type = %d, src = %d, next_hop = %d, dest = %d\r\n", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
          send_packet(tx_buf);
          RMSG = NULL;
        } else {
          printf("[TX-RMSG] broadcasting rreq...\r\n");
          broadcast_id++;
          // construct RREQ message
          aodvrreq.type = 1;
          aodvrreq.broadcast_id = broadcast_id;
          aodvrreq.src = aodvmsg.src;
          aodvrreq.src_seq_num = 1; 
          aodvrreq.dest = aodvmsg.dest;
          aodvrreq.dest_seq_num = dest_seq_num;
          aodvrreq.hop_count = 1;
          // set flag for tx_task, so tx_task can broadcast!
          RREQ = &aodvrreq;
        }
      }
    }
    
    if (RREQ) {
      printf("[TX-RREQ] inside the condition.\r\n");
      if (strcmp(WHOAMI, "source") == 0) {
        node_seq_num++;
      }
      aodvrreq = *RREQ;
      printf("[TX-RREQ] type = %d, broadcast_id = %d, src = %d, src_seq_num = %d, src_seq_num = %d, \
        dest = %d, dest_seq_num = %d, hop_count = %d\r\n", aodvrreq.type, aodvrreq.broadcast_id, 
        aodvrreq.src, aodvrreq.src_seq_num, aodvrreq.dest, aodvrreq.dest_seq_num, aodvrreq.hop_count);
      pack_aodv_rreq(tx_buf, aodvrreq);
      broadcast_rreq(tx_buf);
      is_broadcasting = 1;
      RREQ = NULL;
    }

    if (RERR) {
      printf("[TX-RERR] inside the condition.\r\n");
      aodvrerr = *RERR;
      pack_aodv_rerr(tx_buf, aodvrerr);
      send_rerr(tx_buf, find_next_hop(aodvrerr.src));
      RERR = NULL;
    }

    nrk_wait_until_next_period();
  }
}


void serial_task()
{
  nrk_sig_t uart_rx_signal;
  nrk_sig_mask_t sm;
  
  AODV_MSG_INFO aodvmsg;
  char c;

  // Get the signal for UART RX
  uart_rx_signal=nrk_uart_rx_signal_get();
  // Register task to wait on signal
  nrk_signal_register(uart_rx_signal);
    
  int ret = -3;
  int msg_seq_no = 0;
  
  while(strcmp(WHOAMI, "destination") != 0) {
    printf("[DEBUG-serial] ready to read...\r\n");
    if (nrk_uart_data_ready(NRK_DEFAULT_UART)) {
      ret = scanf("%c", &c);
      msg[0] = c;
      aodvmsg.type = 0;
      aodvmsg.src = SRC_ADDR;
      aodvmsg.dest = DEST_ADDR;
      aodvmsg.msg_seq_no = msg_seq_no++;
      aodvmsg.msg_len = 1;
      aodvmsg.msg = msg;
      RMSG = &aodvmsg;
      printf("[DEBUG-serial] ret: %d || char: %c\r\n", ret, c);
    } else {
      printf("[DEBUG-serial] ready to wait...\r\n");
      // Suspend until UART data arrives
      sm = nrk_event_wait(SIG(uart_rx_signal));
      if (sm != SIG(uart_rx_signal)) {
        nrk_kprintf(PSTR("[DEBUG-serial] UART RX signal error!"));
      }
    }
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

  SERIAL_TASK.task = serial_task;
  nrk_task_set_stk(&SERIAL_TASK, serial_task_stack, NRK_APP_STACKSIZE);
  SERIAL_TASK.prio = 2;
  SERIAL_TASK.FirstActivation = TRUE;
  SERIAL_TASK.Type = BASIC_TASK;
  SERIAL_TASK.SchType = PREEMPTIVE;
  SERIAL_TASK.period.secs = 0;
  SERIAL_TASK.period.nano_secs = 500 * NANOS_PER_MS;
  SERIAL_TASK.cpu_reserve.secs = 1;
  SERIAL_TASK.cpu_reserve.nano_secs = 0;
  SERIAL_TASK.offset.secs = 0;
  SERIAL_TASK.offset.nano_secs = 0;
  nrk_activate_task (&SERIAL_TASK);
  
  printf ("[DEBUG] Tasks creations are done.\r\n");
}
