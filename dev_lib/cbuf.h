#ifndef CBUF_H
#define CBUF_H
#include <stdint.h>
typedef struct {
    int16_t * const buffer;
    uint16_t head;
    uint16_t tail;
    volatile uint16_t length;
    const uint16_t maxindex;
} cbuf_s16;

typedef struct {
    uint16_t * const buffer;
    uint16_t head;
    uint16_t tail;
    volatile uint16_t length;
    const uint16_t maxindex;
} cbuf_u16;

typedef struct {
    uint8_t * const buffer;
    uint16_t head;
    uint16_t tail;
    volatile uint16_t length;
    const uint16_t maxindex;
} cbuf_u8;



int8_t buf_push_u16(cbuf_u16 *c, uint16_t data);
int8_t buf_pop_u16(cbuf_u16 *c, uint16_t *data);
int8_t buf_push_s16(cbuf_s16 *c, int16_t data);
int8_t buf_pop_s16(cbuf_s16 *c, int16_t *data);

int8_t buf_push_u8(cbuf_u8 *c, uint8_t data);
int8_t buf_pop_u8(cbuf_u8 *c, uint8_t *data);

//New ring buffer realizaion

typedef struct {
  uint8_t *data;
  uint32_t size;
  uint32_t head;
  uint32_t tail;
  uint32_t length;
} rb_u8;

typedef struct {
  uint16_t *data;
  uint32_t size;
  uint32_t head;
  uint32_t tail;
  uint32_t length;
} rb_u16;


uint8_t rb_u8_push(rb_u8 *ring, uint8_t data);
uint8_t rb_u16_push(rb_u16 *ring, uint16_t data);
uint8_t rb_u8_pop(rb_u8 *ring, uint8_t *data);
uint8_t rb_u16_pop(rb_u16 *ring, uint16_t *data);

//

#endif
