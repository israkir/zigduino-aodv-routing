#ifndef _AODV_H
#define _AODV_H

#include <include.h>
#include <nrk.h>

typedef struct _routing_entry{
  uint8_t dest;
  uint8_t next_hop;
  uint8_t hop_count;
  uint8_t dest_seq_num;
  uint8_t neighbor_len;
  uint8_t* neighbors;
  uint8_t lifespan;
  int8_t ssnr2;
  struct _routing_entry *next;
} ROUTING_ENTRY; 

typedef struct{
  uint8_t type;
  uint8_t src;
  uint8_t next_hop;
  uint8_t dest;
  uint8_t msg_len;
  uint8_t* msg;
} AODV_MSG_INFO;

typedef struct{
  uint8_t type;
  uint8_t broadcast_id;
  uint8_t src;
  uint8_t src_seq_num;
  uint8_t dest;
  uint8_t dest_seq_num;
  uint8_t hop_count;
} AODV_RREQ_INFO;

typedef struct{
  uint8_t type;
  uint8_t src;
  uint8_t dest;
  uint8_t dest_seq_num;
  uint8_t hop_count;
  uint8_t lifespan;
} AODV_RREP_INFO;

extern uint8_t node_addr;
extern uint8_t node_seq_num;
extern uint8_t table_size;
extern ROUTING_ENTRY routing_table[];
extern AODV_RREQ_INFO rreq_buffer[];

int8_t init_aodv();
uint8_t get_msg_type(uint8_t* rx_buf);
void unpack_aodv_msg(uint8_t* rx_buf, AODV_MSG_INFO* aodvmsg, uint8_t* msg);
void unpack_aodv_RREQ(uint8_t* rx_buf, AODV_RREQ_INFO* aodvrreq);
void pack_aodv_RREQ(uint8_t* tx_buf, AODV_RREQ_INFO aodvrreq);
void pack_aodv_msg(uint8_t* tx_buf, AODV_MSG_INFO aodvmsg);
void repack_forward_msg(uint8_t* buf, AODV_MSG_INFO aodvmsg, uint8_t next_hop);
int8_t add_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr);
int8_t remove_routing_entry();
void print_routing_table();
uint8_t find_index(uint8_t dest, uint8_t next_hop);
uint8_t find_next_hop(uint8_t dest);
void send_packet(uint8_t *tx_buf, uint8_t length);
void broadcast_packet(uint8_t *tx_buf, uint8_t length);
void set_routing_table();
uint8_t getuint_8();
void update_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr);

#endif
