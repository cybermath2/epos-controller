#pragma once

#include <stdint.h>
#include <stddef.h>

void set(const struct cob_id *id, void *port, uint16_t node_id, void *p, size_t n);
void get(void *buf, size_t n, void *port, uint16_t node_id, const struct cob_id *id);
void die(uint32_t err, const char *what, ...);

void driver_info_dump(void);
void node_info_dump(void *port, uint16_t node_id);
void str_motor_type(char *buf, size_t n, uint16_t motor_type);

// dimension (kg, s, ...)
void str_unit_dim(char *buf, size_t n, uint8_t dimension);

// notation (deci, centi, ...)
void str_unit_not(char *buf, size_t n, uint8_t notation);

// whole unit (prefix, numerator, denumerator)
void str_unit(char *buf, size_t n, uint32_t unit);
