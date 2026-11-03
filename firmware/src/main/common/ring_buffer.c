/*
 * ring_buffer.c
 *
 *  Created on: 2020. 12. 16.
 *      Author: WANG
 */

#include "ring_buffer.h"

void qbufferInit(void)
{

}

bool qbufferCreate(qbuffer_t *p_node, uint8_t *p_buf, uint32_t length)
{
  if (p_node == NULL || p_buf == NULL || length < 2)
  {
      return false;
  }

  p_node->in    = 0;
  p_node->out   = 0;
  p_node->len   = length;
  p_node->size  = 1;
  p_node->p_buf = p_buf;

  return true;
}

bool qbufferCreateBySize(qbuffer_t *p_node, uint8_t *p_buf, uint32_t size, uint32_t length)
{
  bool ret = true;

  p_node->in    = 0;
  p_node->out   = 0;
  p_node->len   = length;
  p_node->size  = size;
  p_node->p_buf = p_buf;

  return ret;
}

bool qbufferWrite(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL ||
      p_node->p_buf == NULL ||
      p_data == NULL ||
      p_node->len < 2 ||
      p_node->size == 0)
  {
      return false;
  }

  for (uint32_t i = 0; i < length; i++)
  {
      uint32_t next_in =
          (p_node->in + 1U) % p_node->len;

      // 버퍼가 가득 찬 경우
      if (next_in == p_node->out)
      {
          return false;
      }

      // 현재 위치에 데이터 저장
      uint8_t *p_buf =
          &p_node->p_buf[p_node->in * p_node->size];

      for (uint32_t j = 0; j < p_node->size; j++)
      {
          p_buf[j] = p_data[j];
      }

      p_data += p_node->size;

      // 쓰기 인덱스 갱신
      p_node->in = next_in;
  }

  return true;
}

bool qbufferRead(qbuffer_t *p_node, uint8_t *p_data, uint32_t length)
{
  if (p_node == NULL ||
      p_node->p_buf == NULL ||
      p_data == NULL ||
      p_node->len < 2 ||
      p_node->size == 0)
  {
      return false;
  }

  for (uint32_t i = 0; i < length; i++)
  {
      // 버퍼가 비어 있는 경우
      if (p_node->out == p_node->in)
      {
          return false;
      }

      // 현재 위치에서 데이터 읽기
      uint8_t *p_buf =
          &p_node->p_buf[p_node->out * p_node->size];

      for (uint32_t j = 0; j < p_node->size; j++)
      {
          p_data[j] = p_buf[j];
      }

      p_data += p_node->size;

      // 읽기 인덱스 갱신
      p_node->out =
          (p_node->out + 1U) % p_node->len;
  }

  return true;
}

uint8_t *qbufferPeekWrite(qbuffer_t *p_node)
{
  return &p_node->p_buf[p_node->in*p_node->size];
}

uint8_t *qbufferPeekRead(qbuffer_t *p_node)
{
  return &p_node->p_buf[p_node->out*p_node->size];
}


uint32_t qbufferAvailable(qbuffer_t *p_node)
{
  uint32_t ret;

  if(p_node->in >= p_node->out)
  {
  	ret = p_node->in - p_node->out;
  }
  else
  {
  	ret = p_node->len + p_node->in - p_node->out;
  }

  //ret = (p_node->len + p_node->in - p_node->out) % p_node->len;

  return ret;
}

bool qbufferTxEmpty(qbuffer_t *p_node)
{
  return p_node->in == p_node->out;
}

bool qbufferTxBytesFree(qbuffer_t *p_node)
{
  uint32_t bytesUsed;

  if (p_node->in >= p_node->out) {
      bytesUsed = p_node->in - p_node->out;
  } else {
      bytesUsed = p_node->size + p_node->in - p_node->out;
  }

  return (p_node->size - 1) - bytesUsed;
}

void qbufferFlush(qbuffer_t *p_node)
{
  p_node->in  = 0;
  p_node->out = 0;
}


