/*
 * ring_buffer.c
 *
 *  Created on: 2020. 12. 16.
 *      Author: WANG
 */

#include "ring_buffer.h"


static bool ringWrite(volatile uint8_t *buf, uint32_t size,
                      volatile uint32_t *head, volatile uint32_t *tail,
                      const uint8_t *p_data, uint32_t length)
{
  for (uint32_t i = 0; i < length; i++)
  {
    uint32_t next = (*head + 1U) % size;

    // 버퍼가 가득 찬 경우
    if (next == *tail)
    {
      return false;
    }

    buf[*head] = p_data[i];
    *head = next;               // 쓰기 인덱스 갱신
  }

  return true;
}

static bool ringRead(volatile uint8_t *buf, uint32_t size,
                     volatile uint32_t *head, volatile uint32_t *tail,
                     uint8_t *p_data, uint32_t length)
{
  for (uint32_t i = 0; i < length; i++)
  {
    // 버퍼가 비어 있는 경우
    if (*tail == *head)
    {
      return false;
    }

    p_data[i] = buf[*tail];
    *tail = (*tail + 1U) % size;  // 읽기 인덱스 갱신
  }

  return true;
}

static uint32_t ringAvailable(uint32_t size, uint32_t head, uint32_t tail)
{
  if (head >= tail)
  {
    return head - tail;
  }

  return size + head - tail;
}

/* ---------- 초기화 ---------- */

bool qbufferCreate(qbuffer_t *p_node,
                   uint8_t *p_rx_buf, uint32_t rx_size,
                   uint8_t *p_tx_buf, uint32_t tx_size)
{
  if (p_node == NULL ||
      p_rx_buf == NULL || rx_size < 2 ||
      p_tx_buf == NULL || tx_size < 2)
  {
    return false;
  }

  p_node->rxBuffer     = p_rx_buf;
  p_node->rxBufferSize = rx_size;
  p_node->rxBufferHead = 0;
  p_node->rxBufferTail = 0;

  p_node->txBuffer     = p_tx_buf;
  p_node->txBufferSize = tx_size;
  p_node->txBufferHead = 0;
  p_node->txBufferTail = 0;

  return true;
}

/* ---------- RX ---------- */

bool qbufferRxWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL || p_node->rxBuffer == NULL || p_data == NULL || p_node->rxBufferSize < 2)
  {
    return false;
  }

  return ringWrite(p_node->rxBuffer, p_node->rxBufferSize,
                   &p_node->rxBufferHead, &p_node->rxBufferTail,
                   p_data, length);
}

bool qbufferRxRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL || p_node->rxBuffer == NULL || p_data == NULL || p_node->rxBufferSize < 2)
  {
    return false;
  }

  return ringRead(p_node->rxBuffer, p_node->rxBufferSize,
                  &p_node->rxBufferHead, &p_node->rxBufferTail,
                  p_data, length);
}

volatile uint8_t *qbufferRxPeekWrite(qbuffer_t *p_node)
{
  return &p_node->rxBuffer[p_node->rxBufferHead];
}

volatile uint8_t *qbufferRxPeekRead(qbuffer_t *p_node)
{
  return &p_node->rxBuffer[p_node->rxBufferTail];
}

uint32_t qbufferRxAvailable(qbuffer_t *p_node)
{
  return ringAvailable(p_node->rxBufferSize, p_node->rxBufferHead, p_node->rxBufferTail);
}

bool qbufferRxEmpty(qbuffer_t *p_node)
{
  return p_node->rxBufferHead == p_node->rxBufferTail;
}

void qbufferRxFlush(qbuffer_t *p_node)
{
  p_node->rxBufferTail = p_node->rxBufferHead;   // 소비자는 Tail만 수정
}

/* ---------- TX ---------- */

bool qbufferTxWrite(qbuffer_t *p_node, const uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL || p_node->txBuffer == NULL || p_data == NULL || p_node->txBufferSize < 2)
  {
    return false;
  }

  return ringWrite(p_node->txBuffer, p_node->txBufferSize,
                   &p_node->txBufferHead, &p_node->txBufferTail,
                   p_data, length);
}

bool qbufferTxRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL || p_node->txBuffer == NULL || p_data == NULL || p_node->txBufferSize < 2)
  {
    return false;
  }

  return ringRead(p_node->txBuffer, p_node->txBufferSize,
                  &p_node->txBufferHead, &p_node->txBufferTail,
                  p_data, length);
}

volatile uint8_t *qbufferTxPeekWrite(qbuffer_t *p_node)
{
  return &p_node->txBuffer[p_node->txBufferHead];
}

volatile uint8_t *qbufferTxPeekRead(qbuffer_t *p_node)
{
  return &p_node->txBuffer[p_node->txBufferTail];
}

uint32_t qbufferTxAvailable(qbuffer_t *p_node)
{
  return ringAvailable(p_node->txBufferSize, p_node->txBufferHead, p_node->txBufferTail);
}

bool qbufferTxEmpty(qbuffer_t *p_node)
{
  return p_node->txBufferHead == p_node->txBufferTail;
}

uint32_t qbufferTxBytesFree(qbuffer_t *p_node)
{
  // 링버퍼는 한 칸을 비워 두므로 최대 사용 가능 = size - 1
  return (p_node->txBufferSize - 1U) - qbufferTxAvailable(p_node);
}

void qbufferTxFlush(qbuffer_t *p_node)
{
  p_node->txBufferTail = p_node->txBufferHead;
}

