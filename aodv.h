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
  uint8_t msg_seq_no;
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

typedef struct{
  uint8_t type;
} AODV_RERR_INFO;

extern nrk_time_t timeout_t;
extern uint8_t node_addr;
extern uint8_t node_seq_num;
extern uint8_t dest_seq_num;
extern uint8_t table_size;
extern uint8_t rreq_buffer_size;
extern ROUTING_ENTRY routing_table[];
extern AODV_RREQ_INFO rreq_buffer[];


uint8_t get_msg_type(uint8_t* rx_buf);
void unpack_aodv_rreq(uint8_t* rx_buf, AODV_RREQ_INFO* aodvrreq);
void unpack_aodv_rrep(uint8_t* tx_buf, AODV_RREP_INFO* aodvrrep);
void unpack_aodv_msg(uint8_t* rx_buf, AODV_MSG_INFO* aodvmsg, uint8_t* msg);
void unpack_aodv_rerr(uint8_t* rx_buf, AODV_RERR_INFO* aodvrerr);

uint8_t pack_aodv_rreq(uint8_t* tx_buf, AODV_RREQ_INFO aodvrreq);
uint8_t pack_aodv_rrep(uint8_t* tx_buf, AODV_RREP_INFO aodvrrep);
uint8_t pack_aodv_msg(uint8_t* tx_buf, AODV_MSG_INFO aodvmsg);
uint8_t pack_aodv_rerr(uint8_t* tx_buf, AODV_RERR_INFO aodvrerr);

void repack_forward_msg(AODV_MSG_INFO* aodvmsg, uint8_t next_hop);

int8_t add_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr);
int8_t update_routing_entry(uint8_t dest, uint8_t next_hop, uint8_t dest_seq_num, uint8_t hop_count, int8_t snr);
void renew_routing_entry(uint8_t dest, uint8_t dest_seq_num);
int8_t clean_routing_table();
void print_routing_table();

uint8_t find_index(uint8_t dest, uint8_t dest_seq_num);
uint8_t find_next_hop(uint8_t dest);
uint8_t find_next_hop_by_ssnr2(uint8_t dest);
uint8_t find_next_hop_by_ssnr2_and_hop_count(uint8_t dest);

uint8_t send_packet(uint8_t *tx_buf, uint8_t length);
uint8_t send_rrep(uint8_t *tx_buf, uint8_t next_hop, uint8_t length);
uint8_t broadcast_rreq(uint8_t *tx_buf, uint8_t length);
uint8_t send_rerr(uint8_t *tx_buf, uint8_t next_hop, uint8_t length);

int8_t add_rreq_to_buffer(AODV_RREQ_INFO* aodvrreq);
int8_t check_rreq_is_valid(AODV_RREQ_INFO* aodvrreq);
void print_rreq_buffer();

#endif
