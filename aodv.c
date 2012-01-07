#include <avr/sleep.h>
#include <ulib.h>
#include <stdio.h>
#include <stdlib.h>
#include <hal.h>
#include <aodv.h>
#include <nrk_error.h>
#include <ctype.h>
#include "my_basic_rf.h"

#define MAX_TABLE_SIZE 32
#define RREQ_BUFFER_SIZE 2

ROUTING_ENTRY routing_table[MAX_TABLE_SIZE];
AODV_RREQ_INFO rreq_buffer[RREQ_BUFFER_SIZE];

uint8_t aodv_id;
uint8_t table_size = 0;
uint8_t rreq_buffer_size = 0;

RF_TX_INFO rfTxInfo;
RF_RX_INFO rfRxInfo;
uint8_t tx_buf[RF_MAX_PAYLOAD_SIZE];
uint8_t rx_buf[RF_MAX_PAYLOAD_SIZE];

uint8_t getuint_8(){
  char str[32];
  int8_t i = 0;
  uint8_t num;
  bool isdigitstr;
  nrk_sig_t uart_rx_signal;
  // Get the signal for UART RX
  uart_rx_signal=nrk_uart_rx_signal_get();
  // Register task to wait on signal
  nrk_signal_register(uart_rx_signal);

  while(1){
    isdigitstr = true;
    i = 0;
    while(1){
      if(nrk_uart_data_ready(NRK_DEFAULT_UART)){
        // Blocking Read that should only be called if you know data is in the buffer
        str[i]=getchar();
        printf("%c", str[i]);
        if(str[i] == '\r'){
          break;
        }
        
        if(!isdigit(str[i])){
          isdigitstr = false;
        }
        i++;
      }else {
        // Suspend until UART data arrives
        nrk_event_wait(SIG(uart_rx_signal));
      }
    }
    str[i] = '\0';
    nrk_kprintf(PSTR("\r\n"));

    if(isdigitstr == true){
      num = atoi(str);
      break;
    }
  }
  return num;
}

void update_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr) {
  uint8_t index = find_index(dest, next_hop);
  if (routing_table[index].dest_seq_num < dest_seq_num) {
    routing_table[index].dest = dest;
    routing_table[index].next_hop = next_hop;
    routing_table[index].dest_seq_num = dest_seq_num;
    routing_table[index].hop_count = hop_count;

    // Update ssnr2
    routing_table[index].ssnr2 = 0.5 * (routing_table[table_size].ssnr2 + snr);
  }
}

int8_t add_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr){
  
  if(table_size < MAX_TABLE_SIZE){
    routing_table[table_size].dest = dest;
    routing_table[table_size].next_hop = next_hop;
    routing_table[table_size].dest_seq_num = dest_seq_num;
    routing_table[table_size].hop_count = hop_count;

    // Update ssnr2
    routing_table[table_size].ssnr2 = 0.5 * (routing_table[table_size].ssnr2 + snr);

    table_size++;
  }
  return 0;
}

int8_t remove_routing_entry(){
  return 0;
}

void print_routing_table(){
  uint8_t i;
  nrk_kprintf(PSTR("d,n,s,h\r\n"));
  for(i=0 ; i<table_size ; i++){
    printf("%d,%d,%d,%d\r\n", routing_table[i].dest, routing_table[i].next_hop, routing_table[i].dest_seq_num, routing_table[i].hop_count);
  }
}

uint8_t get_msg_type(uint8_t* rx_buf){
  return rx_buf[0];
}

void unpack_aodv_RREQ(uint8_t* rx_buf, AODV_RREQ_INFO* aodvrreq){
  aodvrreq->type = rx_buf[0];
  aodvrreq->broadcast_id = rx_buf[1];
  aodvrreq->src = rx_buf[2];
  aodvrreq->dest = rx_buf[3];
  aodvrreq->lifespan = rx_buf[4];
  aodvrreq->hop_count = rx_buf[5];
}

void pack_aodv_RREQ(uint8_t* tx_buf, AODV_RREQ_INFO aodvrreq){
  tx_buf[0] = aodvrreq.type;
  tx_buf[1] = aodvrreq.broadcast_id;
  tx_buf[2] = aodvrreq.src;
  tx_buf[3] = aodvrreq.dest;
  tx_buf[4] = aodvrreq.lifespan;
  tx_buf[5] = aodvrreq.hop_count;
}

void unpack_aodv_msg(uint8_t* rx_buf, AODV_MSG_INFO* aodvmsg, uint8_t* msg){
  aodvmsg->type = rx_buf[0];
  aodvmsg->src  = rx_buf[1];
  aodvmsg->next_hop = rx_buf[2];
  aodvmsg->dest = rx_buf[3];
  aodvmsg->length = rx_buf[4];
  aodvmsg->msg = msg;
  memcpy(msg,rx_buf+5,aodvmsg->length);
}

void pack_aodv_msg(uint8_t* tx_buf, AODV_MSG_INFO aodvmsg){
  printf("\r\npack msg type = %o, src = %o\r\n",aodvmsg.type,aodvmsg.src);
  tx_buf[0] = aodvmsg.type;
  tx_buf[1] = aodvmsg.src;
  tx_buf[2] = aodvmsg.next_hop;
  tx_buf[3] = aodvmsg.dest;
  tx_buf[4] = aodvmsg.length;
  memcpy(tx_buf+5, aodvmsg.msg, aodvmsg.length);
}

void repack_forward_msg(uint8_t* buf, AODV_MSG_INFO aodvmsg, uint8_t next_hop){
  buf[2] = next_hop;
}

uint8_t find_index(uint8_t dest, uint8_t next_hop){
  uint8_t i;
  for(i = 0; i < table_size; i++){
    if(routing_table[i].dest == dest && routing_table[i].next_hop == next_hop){
      return i;
    }
  }
  return -1; // did not find in routing table
}

uint8_t find_next_hop(uint8_t dest){
  uint8_t i;
  for(i = 0; i < table_size ; i++){
    if(routing_table[i].dest == dest){
      return routing_table[i].next_hop;
    }
  }
  return 0; // 0 => did not find in routing table
}

void set_routing_table()
{
  uint8_t i, table_size, dest, nexthop, destseq, hopcount;
  nrk_kprintf(PSTR ("Enter id (dont use id 0):\r\n"));
  //id 0 is reserved for the meaning of not exist.
  aodv_id = getuint_8();
  printf("id = %d\r\n", aodv_id);
  nrk_kprintf(PSTR ("Enter table size:\r\n"));
  table_size = getuint_8();
  printf("table size = %d\r\n", table_size);
  for(i = 0; i < table_size; ++i){
    nrk_kprintf (PSTR ("Enter dest nexthop destseq hopcount\r\n"));
    dest = getuint_8();
    nexthop = getuint_8();
    destseq = getuint_8();
    hopcount = getuint_8();
    printf("%d %d %d %d\r\n", dest, nexthop, destseq, hopcount);
    add_routing_entry(dest, nexthop, destseq, hopcount, 0);
  }
  print_routing_table();
}

void broadcast_packet(uint8_t *tx_buf, uint8_t length) {
  uint8_t val;

  printf("txpacket type = %d, src = %d, next_hop = %d, dest = %d\r\n", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
  rfTxInfo.pPayload = tx_buf;
  rfTxInfo.length = length;
  rfTxInfo.destAddr = 0xffff; // broadcast by default
  rfTxInfo.cca = 0;
  rfTxInfo.ackRequest = 1;

  //printf( "Sending\r\n" );
  if(rf_tx_packet(&rfTxInfo) != 1){
    nrk_kprintf (PSTR ("@@@ RF_TX ERROR @@@\r\n"));
  }else{
    nrk_kprintf (PSTR ("--- RF_TX ACK!! ---\r\n"));
  }

  nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
}

void send_packet(uint8_t *tx_buf, uint8_t length){
  uint8_t val;

  printf("txpacket type = %d, src = %d, next_hop = %d, dest = %d\r\n", tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
  rfTxInfo.pPayload = tx_buf;
  rfTxInfo.length = length;
  rfTxInfo.destAddr = tx_buf[2]; // next_hop
  rfTxInfo.cca = 0;
  rfTxInfo.ackRequest = 1;

  //printf( "Sending\r\n" );
  if(rf_tx_packet(&rfTxInfo) != 1){
    nrk_kprintf (PSTR ("@@@ RF_TX ERROR @@@\r\n"));
  }else{
    nrk_kprintf (PSTR ("--- RF_TX ACK!! ---\r\n"));
  }

  nrk_kprintf (PSTR ("Tx task sent data!\r\n"));
}


int8_t add_rreq_to_buffer(AODV_RREQ_INFO* aodvrreq) {
  if (rreq_buffer_size < RREQ_BUFFER_SIZE) {
    rreq_buffer[rreq_buffer_size].type = aodvrreq.type;
    rreq_buffer[rreq_buffer_size].broadcast_id = aodvrreq.broadcast_id;
    rreq_buffer[rreq_buffer_size].src = aodvrreq.src;
    rreq_buffer[rreq_buffer_size].src_seq_num = aodvrreq.src_seq_num;
    rreq_buffer[rreq_buffer_size].dest = aodvrreq.dest;
    rreq_buffer[rreq_buffer_size].dest_seq_num = aodvrreq.dest_seq_num;
    rreq_buffer[rreq_buffer_size].hop_count = aodvrreq.hop_count;
    rreq_buffer_size++;
    return 0;
  } else {
    printf("ERROR: RREQ_BUFFER_SIZE exceeded!!!");
    return -1;
  }
}

int8_t check_rreq_is_valid(AODV_RREQ_INFO* aodvrreq) {
  // check node received a RREQ with the same broadcast_id & source addr
  // if it did, return -1 (drop the rreq packet); return 0, otherwise.
  if (rreq_buffer_size == 0) {
    return 0;
  } else {
    int i;
    for (i=0; i<rreq_buffer_size; i++) {
      if ((rreq_buffer[i].broadcast_id == aodvrreq.broadcast_id) && (rreq_buffer[i].src == aodvrreq.src)) {
        return -1;
      } else {
        return 0;
      }
    }
  }
}