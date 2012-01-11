// TODO: link breakage handling
// TODO: multiple RREQ message before adding a route
// snr based routing table update (implemented but haven't tested yet)
// error handling for RX/TX ERROR and CRC Failure (fixed by increasing stack
// size)
// intermediate node data buffering (fixed)

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

#define MAX_NEW_MSG_LEN 128
#define MAX_MSG_LEN 128
#define MAX_RETRY 10

nrk_task_type RX_TASK;
NRK_STK rx_task_stack[NRK_APP_STACKSIZE];
void rx_task (void);

nrk_task_type TX_TASK;
NRK_STK tx_task_stack[NRK_APP_STACKSIZE];
void tx_task (void);

nrk_task_type SERIAL_TASK;
NRK_STK serial_task_stack[NRK_APP_STACKSIZE];
void serial_task(void);

void nrk_create_taskset();

nrk_sem_t *buffer_semaphore;

uint8_t rxmsg[MAX_MSG_LEN];
uint8_t msg[MAX_MSG_LEN];
uint8_t new_msg[MAX_NEW_MSG_LEN];

uint8_t new_msg_len = 0;
uint8_t msg_seq_no = 0;
uint8_t source_broadcasting = 0;
uint8_t retry = 0;

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

int light_val;

void init_srand_seed() {
  uint8_t val;
  uint8_t buf;
  int8_t fd;

  fd = nrk_open(ADC_DEV_MANAGER, READ);
  if (fd==NRK_ERROR) nrk_kprintf(PSTR("Failed to open adc driver\r\n"));
  val = nrk_set_status(fd, ADC_CHAN, 0);
  val = nrk_read(fd, &buf, 1);
  light_val = buf;
  nrk_close(fd);
  printf("light_val = %d\r\n", light_val);
}

int main ()
{
  nrk_setup_ports ();
  nrk_setup_uart (UART_BAUDRATE_115K2);
  
  // Register the ADC device driver
  uint8_t val;
  val = nrk_register_driver( &dev_manager_adc,ADC_DEV_MANAGER);
  if(val==NRK_ERROR) nrk_kprintf( PSTR("Failed to load my ADC driver\r\n") );

  timeout_t.secs = 1;
  timeout_t.nano_secs = 0;
  buffer_semaphore = nrk_sem_create(1,4);
  
  // init a unique id for this node
  if (WHOAMI == "source") {
    node_addr = SRC_ADDR;
  } else if (WHOAMI == "destination") {
    node_addr = DEST_ADDR;
  } else {
    init_srand_seed();
    srand(light_val);
    printf("rand() = %d\r\n", rand());
    node_addr = rand() % 100 + 7;
    /*node_addr = 111;*/
  }
  printf("[DEBUG-tx_task] My addr is: %d\r\n", node_addr);

  
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
        memset(rxmsg, 0, MAX_MSG_LEN);
        unpack_aodv_msg (local_rx_buf, &aodvmsg, rxmsg);
        printf("[RX-RMSG] type = %d, src = %d, nexthop = %d, dest = %d, msg_len = %d, msg = %s, srcAddr = %d\r\n", aodvmsg.type, aodvmsg.src, aodvmsg.next_hop, aodvmsg.dest, aodvmsg.msg_len, aodvmsg.msg, rfRxInfo.srcAddr);

        if(aodvmsg.next_hop == node_addr) {
          // this AODV msg is for this node, so process it!
          if(aodvmsg.dest == node_addr){
            // this node is destination, so print received packet
            printf("[RX-DATA] ");
            for(i=0; i<aodvmsg.msg_len; i++ ) printf("[%c]", aodvmsg.msg[i]);
            printf("\r\n");
            memset(rxmsg, 0, aodvmsg.msg_len);
          } else {
            // this node is not destination, so send it to neighbor
            if((next_hop = find_next_hop_by_ssnr2_and_hop_count(aodvmsg.dest)) != 0){
              printf("[RX-DATA] sendmsg to %d\r\n", next_hop);
              repack_forward_msg(&aodvmsg, next_hop);
              RMSG = &aodvmsg;
            } else {
              aodvrerr.type = 4;
              RERR = &aodvrerr;
            }
          }
        }
      } else if(type == 1) { // RREQ
        unpack_aodv_rreq(local_rx_buf, &aodvrreq);
        printf("[RX-RREQ] type = %d, broadcast_id = %d, src = %d, src_seq_num = %d, dest = %d, dest_seq_num = %d, hop_count = %d, srcAddr = %d\r\n", aodvrreq.type, aodvrreq.broadcast_id, aodvrreq.src, aodvrreq.src_seq_num, aodvrreq.dest, aodvrreq.dest_seq_num, aodvrreq.hop_count, rfRxInfo.srcAddr);

        // DEBUG PURPOSE for 3 nodes, so it will force routing through the
        // third intermediate node
        /*
        if (strcmp(WHOAMI, "destination") == 0 && rfRxInfo.srcAddr == SRC_ADDR) {
          continue;
        }
        */

        // valid: this node did not received a RREQ with greater broadcast_id & source addr
        if (check_rreq_is_valid(&aodvrreq) != -1) {
          printf("[RX-RREQ] check is valid - adding the request to routing entry\r\n");
          // create inverse routing entry
          add_routing_entry(aodvrreq.src, rfRxInfo.srcAddr, aodvrreq.src_seq_num, aodvrreq.hop_count, rfRxInfo.rssi); 
          print_routing_table();

          // this node neighbor of destination, so RREP!
          if ((aodvrreq.dest == find_next_hop_by_ssnr2_and_hop_count(aodvrreq.dest)) || aodvrreq.dest == node_addr) {
            printf("[RX-RREQ] this node is either destination or neighbor of the destination.\r\n");
            if ((dest_seq_num < aodvrreq.dest_seq_num) || (aodvrreq.dest_seq_num == 0)) {
              // update destination sequence number
              dest_seq_num = aodvrreq.dest_seq_num;

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
          else {
            printf("[RX-RREQ] increase hop count\r\n");
            aodvrreq.hop_count++;

            printf("[RX-RREQ] broadcast rreq message\r\n");
            RREQ = &aodvrreq;
            RREP = NULL;
          }
        }
      } else if(type == 2) { // RREP
        unpack_aodv_rrep (local_rx_buf, &aodvrrep);
        printf("[RX-RREP] type = %d, src = %d, dest = %d, dest_seq_num = %d, hop_count = %d, lifespan = %d, srcAddr = %d\r\n", aodvrrep.type, aodvrrep.src, aodvrrep.dest, aodvrrep.dest_seq_num, aodvrrep.hop_count, aodvrrep.lifespan, rfRxInfo.srcAddr);

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
            source_broadcasting = 0;
            RREP = NULL;
          }
        } else {
          printf("[RX-RREP] invalid/outdated rrep message\r\n");
          RREQ = NULL;
          RREP = NULL;
        }
      } else if(type == 3) { // RERR
        unpack_aodv_rerr (local_rx_buf, &aodvrerr);
        printf("[RX-RERR] type = %d, srcAddr = %d\r\n", aodvrerr.type, rfRxInfo.srcAddr);

        if (WHOAMI == "source") {
          RERR = NULL;
          printf("[TX-RMSG] broadcasting rreq...\r\n");
          broadcast_id++;
          // construct RREQ message
          aodvrreq.type = 1;
          aodvrreq.broadcast_id = broadcast_id;
          aodvrreq.src = SRC_ADDR;
          aodvrreq.src_seq_num = node_seq_num; 
          aodvrreq.dest = DEST_ADDR;
          aodvrreq.dest_seq_num = dest_seq_num;
          aodvrreq.hop_count = 1;
          // set flag for tx_task, so tx_task can broadcast!
          RREQ = &aodvrreq;
        }
        else {
          RERR = &aodvrerr;
          RREQ = NULL;
        }
        source_broadcasting = 0;
        
        //if (WHOAMI == "source") clean_routing_table();
      
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
    } else if(n == NRK_ERROR) {
      printf( "[DEBUG-RX] CRC failure!\r\n" );
      // TODO: restart rx as it might be hardward failure
      // PLAN A
      rf_power_down();
      rf_power_up();

      // PLAN B
      /*rf_rx_off();*/
      /*rf_rx_on();*/
      /*rf_polling_rx_on();*/
    }
  }
}


void tx_task ()
{
  AODV_MSG_INFO aodvmsg;
  AODV_RREQ_INFO aodvrreq;
  AODV_RREP_INFO aodvrrep;
  AODV_RREP_INFO aodvrack;
  AODV_RERR_INFO aodvrerr;

  uint8_t i;

  while(!rf_ok) {
    nrk_wait_until_next_period();
  }
  printf ("[DEBUG-TX] tx_task PID=%d\r\n", nrk_get_pid ());

  while (1) {
    if (new_msg_len > 0) {
      // critical section start
      nrk_sem_pend(buffer_semaphore);
      printf("[TX] new_msg_len = %d\r\n", new_msg_len);
      aodvmsg.msg_len = new_msg_len;
      memcpy(msg, new_msg, new_msg_len);
      printf("[TX] msg = ");
      for(i=0; i<new_msg_len;i++) printf("%d", msg[i]);
      printf("\r\n");
      memset(new_msg, 0, new_msg_len);
      new_msg_len = 0;
      nrk_sem_post(buffer_semaphore);
      // critical section end
      aodvmsg.type = 0;
      aodvmsg.src = SRC_ADDR;
      aodvmsg.dest = DEST_ADDR;
      aodvmsg.msg_seq_no = msg_seq_no++;
      aodvmsg.msg = msg;
      RMSG = &aodvmsg;
    }

    if (RERR) {
      printf("[TX-RERR] inside the condition.\r\n");
      aodvrerr = *RERR;
      uint8_t len = pack_aodv_rerr(tx_buf, aodvrerr);
      // Keep sending until ACK received
      while (send_rerr(tx_buf, find_next_hop_by_ssnr2_and_hop_count(SRC_ADDR), len) != 1) {
        nrk_wait(timeout_t);
        printf("sending rerr...");
      }
      RERR = NULL;
    }
    
    if (RREP) {
      printf("[TX-RREP] inside the condition.\r\n");
      if (strcmp(WHOAMI, "destination") == 0) {
        node_seq_num++;
      }
      aodvrrep = *RREP;
      uint8_t len = pack_aodv_rrep(tx_buf, aodvrrep);
      // Keep sending until ACK received
      while (send_rrep(tx_buf, find_next_hop_by_ssnr2_and_hop_count(aodvrrep.src), len) != 1 && retry < MAX_RETRY) {
        nrk_wait(timeout_t);
        printf("sending rrep...");
        retry++;
      }
      if (retry == MAX_RETRY) {
        retry = 0;
        aodvrerr.type = 4;
        RERR = &aodvrerr;
      }
      else {
      }
      
      RREP = NULL;
    }

    if (RMSG) {
      printf("[TX-RMSG] inside the condition.\r\n");
      aodvmsg = *RMSG;  
      if (!source_broadcasting || WHOAMI != "source") {
        if((aodvmsg.next_hop = find_next_hop_by_ssnr2_and_hop_count(aodvmsg.dest)) != 0){
          uint8_t len = pack_aodv_msg(tx_buf, aodvmsg);
          printf("[TX-RMSG] txpacket type = %d, src = %d, next_hop = %d, dest = %d\r\n", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
          
          // Keep sending until ACK received
          while (send_packet(tx_buf, len) != 1 && retry < MAX_RETRY) {
            nrk_wait(timeout_t);
            printf("sending message...");
            retry++;
          }
          if (retry == MAX_RETRY) {
            retry = 0;
            if (WHOAMI == "intermediate") {
              aodvrerr.type = 4;
              RERR = &aodvrerr;
            }
            else if (WHOAMI == "source") {
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
          else {
          }

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
      printf("[TX-RREQ] type = %d, broadcast_id = %d, src = %d, src_seq_num = %d, dest = %d, dest_seq_num = %d, hop_count = %d\r\n", aodvrreq.type, aodvrreq.broadcast_id, aodvrreq.src, aodvrreq.src_seq_num, aodvrreq.dest, aodvrreq.dest_seq_num, aodvrreq.hop_count);
      uint8_t len = pack_aodv_rreq(tx_buf, aodvrreq);
      // Keep sending until RREP received
      source_broadcasting = 1;
      if (WHOAMI == "source") {
        while (source_broadcasting) {
          broadcast_rreq(tx_buf, len);
          nrk_wait(timeout_t);
          printf("rebroadcasting...");
        }
      }
      else if {
        if (broadcast_rreq(tx_buf, len) != -1) {
          nrk_wait(timeout_t);
          printf("rebroadcasting...");
        }
      }
      RREQ = NULL;
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
  
  while(strcmp(WHOAMI, "destination") != 0) {
    printf("[DEBUG-serial] ready to read...\r\n");
    if (nrk_uart_data_ready(NRK_DEFAULT_UART)) {
      ret = scanf("%c", &c);
      // critical section start
      nrk_sem_pend(buffer_semaphore);
      new_msg[new_msg_len] = c;
      new_msg_len++;
      nrk_sem_post(buffer_semaphore);
      // critical section end
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
