/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Copyright (c) 2023-2026 akd (akd1608@mailbox.org)
 */
#ifndef LIB_PID_H
#define LIB_PID_H

#include "pid_controller_ex.h"

typedef enum {
	PID_TYPE_CLASSIC = 0,
	PID_TYPE_SPEED_UP,
} pid_type_t;

typedef struct {
	pid_type_t type;
	pid_param_t param;
	pid_workset_t set;
	pid_constrain_t sum_limit;
	pid_constrain_t output_limit;

	double error[PID_Qn];
	double set_point;
	double last_value; /* last output, limited */
	double raw_value;  /* velocity form: output before the limit */

	double alpha;

	uint64_t last_stamp;

} pid_controller_t;

/* Clears the whole controller.  Call it first, then set limits and gains. */
int pid_init(pid_controller_t *pid, pid_type_t type);
/* Clears sum, error history and last output.  Gains and limits stay. */
int pid_reset(pid_controller_t *pid);
int pid_run(pid_controller_t *pid, double input, double *output);
/* Without a call there is no limit.  min == max holds the value there, so 0, 0
 * switches the output off.  min > max returns -EINVAL. */
int pid_setOutputConstrain(pid_controller_t *pid, double min, double max);
int pid_setSumConstrins(pid_controller_t *pid, double min, double max);
int pid_setSetPoint(pid_controller_t *pid, double set_point);
int pid_tuneParam(pid_controller_t *pid, double kp, double ki, uint32_t sample_time);
int pid_tuneDiffParam(pid_controller_t *pid, double kd, double kd2, double kd3);

#endif
