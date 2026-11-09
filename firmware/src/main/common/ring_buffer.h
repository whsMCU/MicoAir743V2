/*
 * ring_buffer.h
 *
 *  Created on: 2020. 12. 16.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_RING_BUFFER_H_
#define SRC_COMMON_CORE_RING_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "def.h"

typedef struct
{
  uint32_t rxBufferSize;
  uint32_t txBufferSize;
  volatile uint8_t *rxBuffer;
  volatile uint8_t *txBuffer;
  volatile uint32_t rxBufferHead;
  volatile uint32_t rxBufferTail;
  volatile uint32_t txBufferHead;
  volatile uint32_t txBufferTail;

} qbuffer_t;


bool qbufferCreate(qbuffer_t *p_node,
                   uint8_t *p_rx_buf, uint32_t rx_size,
                   uint8_t *p_tx_buf, uint32_t tx_size);

bool     qbufferRxWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length);
bool     qbufferRxRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length);
volatile uint8_t *qbufferRxPeekWrite(qbuffer_t *p_node);
volatile uint8_t *qbufferRxPeekRead(qbuffer_t *p_node);
uint32_t qbufferRxAvailable(qbuffer_t *p_node);
bool     qbufferRxEmpty(qbuffer_t *p_node);
void     qbufferRxFlush(qbuffer_t *p_node);

bool     qbufferTxWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length);
bool     qbufferTxRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length);
volatile uint8_t *qbufferTxPeekWrite(qbuffer_t *p_node);
volatile uint8_t *qbufferTxPeekRead(qbuffer_t *p_node);
uint32_t qbufferTxAvailable(qbuffer_t *p_node);
bool     qbufferTxEmpty(qbuffer_t *p_node);
uint32_t qbufferTxBytesFree(qbuffer_t *p_node);
void     qbufferTxFlush(qbuffer_t *p_node);

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_CORE_RING_BUFFER_H_ */
