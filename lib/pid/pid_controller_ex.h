/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Copyright (c) 2023-2026 akd (akd1608@mailbox.org)
 */
#ifndef LIB_PID_EX_H
#define LIB_PID_EX_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
	PID_KD1 = 0,
	PID_KD2,
	PID_KD3,
	PID_KDn
} pid_kd_t;

typedef enum {
	PID_Q0 = 0,
	PID_Q1,
	PID_Q2,
	PID_Q3,
	PID_Q4,
	PID_Qn
} pid_q_t;

typedef enum {
	PID_ERROR0,
	PID_ERROR1,
	PID_ERROR2,
	PID_ERROR3,
	PID_ERROR4,
	PID_ERRORn,
} pid_error_t;


/*
 * K-Parameter for pid
 * includes ki, kp, kd[N]
 * sample time and q
 */
typedef struct {
	double kp;
	double ki;
	double kd[PID_KDn];
	double q[PID_Qn];
	uint32_t sample_time;    /* time of the last step (ms)  */
	uint32_t sample_nominal; /* time set when tuning (ms)   */
} pid_param_t;


/*
 * constrain parameter for pid as set
 * consists of min and max set of doubles
 */
typedef struct {
	double min;
	double max;
} pid_constrain_t;

typedef struct {
	double sum;
	double derivative[PID_KDn];
} pid_workset_t;

#endif
