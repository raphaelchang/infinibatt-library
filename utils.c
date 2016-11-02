#include "utils.h"
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <string.h>

static volatile int sys_lock_cnt = 0;

/*
 * Fast approximation of sin and cos. Input is in degrees.
 */
void utils_sincos(float angle, float *sin, float *cos) {
    angle *= M_PI / 180.0;
    //always wrap input angle to -PI..PI
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    while (angle >  M_PI) {
        angle -= 2.0 * M_PI;
    }

    //compute sine
    if (angle < 0.0) {
        *sin = 1.27323954 * angle + 0.405284735 * angle * angle;

        if (*sin < 0.0) {
            *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225 * (*sin * *sin - *sin) + *sin;
        }
    } else {
        *sin = 1.27323954 * angle - 0.405284735 * angle * angle;

        if (*sin < 0.0) {
            *sin = 0.225 * (*sin * -*sin - *sin) + *sin;
        } else {
            *sin = 0.225 * (*sin * *sin - *sin) + *sin;
        }
    }

    // compute cosine: sin(x + PI/2) = cos(x)
    angle += 0.5 * M_PI;
    if (angle >  M_PI) {
        angle -= 2.0 * M_PI;
    }

    if (angle < 0.0) {
        *cos = 1.27323954 * angle + 0.405284735 * angle * angle;

        if (*cos < 0.0) {
            *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225 * (*cos * *cos - *cos) + *cos;
        }
    } else {
        *cos = 1.27323954 * angle - 0.405284735 * angle * angle;

        if (*cos < 0.0) {
            *cos = 0.225 * (*cos * -*cos - *cos) + *cos;
        } else {
            *cos = 0.225 * (*cos * *cos - *cos) + *cos;
        }
    }
}

float utils_atan2(float y, float x) {
    float abs_y = fabsf(y) + 1e-10;
    float res;

    if (x >= 0) {
	float r = (x - abs_y) / (x + abs_y);
	float rsq = r * r;
	res = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
    } else {
	float r = (x + abs_y) / (abs_y - x);
	float rsq = r * r;
	res = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
    }

    if (y < 0) {
	return(-res);
    } else {
	return(res);
    }
}

bool utils_saturate_vector_2d(float *x, float *y, float max) {
    bool retval = false;
    float mag = sqrtf(*x * *x + *y * *y);
    max = fabsf(max);

    if (mag < 1e-10) {
        mag = 1e-10;
    }

    if (mag > max) {
        const float f = max / mag;
        *x *= f;
        *y *= f;
        retval = true;
    }

    return retval;
}

/**
 * A system locking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysLock and chSysLockFromIsr are the same
 * for this port.
 */
void utils_sys_lock_cnt(void) {
    if (!sys_lock_cnt) {
        chSysLock();
    }
    sys_lock_cnt++;
}

/**
 * A system unlocking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysUnlock and chSysUnlockFromIsr are the same
 * for this port.
 */
void utils_sys_unlock_cnt(void) {
    if (sys_lock_cnt) {
        sys_lock_cnt--;
        if (!sys_lock_cnt) {
            chSysUnlock();
        }
    }
}

void utils_norm_angle_rad(float *angle) {
    while (*angle < -M_PI) {
        *angle += 2.0 * M_PI;
    }

    while (*angle >  M_PI) {
        *angle -= 2.0 * M_PI;
    }
}

int16_t utils_parse_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res =   ((uint16_t) buffer[*index]) << 8 |
        ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t utils_parse_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res =  ((uint16_t) buffer[*index]) << 8 |
        ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t utils_parse_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res =   ((uint32_t) buffer[*index]) << 24 |
        ((uint32_t) buffer[*index + 1]) << 16 |
        ((uint32_t) buffer[*index + 2]) << 8 |
        ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t utils_parse_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res = ((uint32_t) buffer[*index]) << 24 |
        ((uint32_t) buffer[*index + 1]) << 16 |
        ((uint32_t) buffer[*index + 2]) << 8 |
        ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

float utils_parse_float32(const uint8_t *buffer, int32_t *index)
{
    float res;
    memcpy(&res, buffer + *index, sizeof(float));
    *index += 4;
    return res;
}

void utils_append_int16(uint8_t* buffer, int16_t number, uint32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void utils_append_uint16(uint8_t* buffer, uint16_t number, uint32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void utils_append_int32(uint8_t* buffer, int32_t number, uint32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void utils_append_uint32(uint8_t* buffer, uint32_t number, uint32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void utils_append_float32(uint8_t *buffer, float value, uint32_t *index) {
    char *bytes = (char*) &value;
    buffer[(*index)++] = bytes[0];
    buffer[(*index)++] = bytes[1];
    buffer[(*index)++] = bytes[2];
    buffer[(*index)++] = bytes[3];
}
