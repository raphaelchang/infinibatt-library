#ifndef _UTILS_H_
#define _UTILS_H_

#define ONE_BY_SQRT3            (0.57735026919)
#define TWO_BY_SQRT3            (2.0f * 0.57735026919)
#define SQRT3_BY_2              (0.86602540378)
#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "ch.h"
#include "hal.h"

void utils_sincos(float angle, float *sin, float *cos);
float utils_atan2(float y, float x);
bool utils_saturate_vector_2d(float *x, float *y, float max);
void utils_sys_lock_cnt(void);
void utils_sys_unlock_cnt(void);
void utils_norm_angle_rad(float *angle);
int16_t utils_parse_int16(const uint8_t *buffer, int32_t *index);
uint16_t utils_parse_uint16(const uint8_t *buffer, int32_t *index);
int32_t utils_parse_int32(const uint8_t *buffer, int32_t *index);
uint32_t utils_parse_uint32(const uint8_t *buffer, int32_t *index);
float utils_parse_float32(const uint8_t *buffer, int32_t *index);
void utils_append_int16(uint8_t* buffer, int16_t number, uint32_t *index);
void utils_append_uint16(uint8_t* buffer, uint16_t number, uint32_t *index);
void utils_append_int32(uint8_t* buffer, int32_t number, uint32_t *index);
void utils_append_uint32(uint8_t* buffer, uint32_t number, uint32_t *index);
void utils_append_float32(uint8_t *buffer, float value, uint32_t *index);

#endif
