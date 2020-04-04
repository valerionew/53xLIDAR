#ifndef WS2812_H
#define WS2812_H

#include "stdint.h"
#include "stddef.h"

void send1Color(uint32_t RGB);
void sendColors(uint32_t * const colorBuffer, size_t len);

#endif
