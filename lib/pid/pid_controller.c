/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Copyright (c) 2023-2026 akd (akd1608@mailbox.org)
 *
 * PID controller, classic and velocity form.
 * Background: https://www.rn-wissen.de/wiki/index.php/Regelungstechnik
 */
#include <pid_controller.h>
#include <pid_controller_ex.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include <stddef.h>

static int pid_updateQ(pid_controller_t *pid);
static int pid_classic(pid_controller_t *pid, double *output);
static int pid_speedUp(pid_controller_t *pid, double *output);
static int pid_handleOldError(pid_controller_t *pid);
static double pid_checkOvershoot(pid_constrain_t constrain, double value);
static double pid_checkWindup(pid_controller_t *pid, double input);

/*
 * Time since the last run.  The first run and a run after a long pause (the
 * caller did not run the controller, e.g. the heater was resting) count as one
 * regular step, so a pause does not end up in the integral as one huge step.
 */
static uint32_t pid_elapsedMs(const pid_controller_t *pid, int64_t now)
{
	uint32_t nominal = pid->param.sample_nominal;
	int64_t elapsed = now - (int64_t)pid->last_stamp;

	if (pid->last_stamp == 0U ||
	    (nominal > 0U && elapsed > (int64_t)nominal * CONFIG_PID_SAMPLE_GAP_FACTOR)) {
		return nominal > 0U ? nominal : 1U;
	}
	/* Two runs in the same millisecond: the derivative divides by this */
	return elapsed > 0 ? (uint32_t)elapsed : 1U;
}

int pid_init(pid_controller_t *pid, pid_type_t type)
{
	pid->type = type;
	pid->alpha = CONFIG_PID_FILTER_COEFFICIENT_ALPHA / 1000.f;
	return 0;
}

int pid_run(pid_controller_t *pid, double input, double *output)
{
	int64_t now = k_uptime_get();
	pid->param.sample_time = pid_elapsedMs(pid, now);
	pid->last_stamp = now;
	pid->error[PID_ERROR0] = pid->set_point - input;
	double temp = 0.f;
    switch (pid->type) {
        case PID_TYPE_CLASSIC:
            pid_classic(pid, &temp);
            break;
        case PID_TYPE_SPEED_UP:
        default:
            pid_speedUp(pid, &temp);
    }
	*output = pid_checkOvershoot(pid->output_limit, temp);
	pid_handleOldError(pid);
	pid->last_value = *output;
	return 0;
}

int pid_setSetPoint(pid_controller_t *pid, double set_point)
{
	pid->set_point = set_point;
#if CONFIG_PID_CONTROLLER_FLUSH_BY_NEW_SETPOINT
	pid->last_value = 0.0;
	for (uint8_t i = 0; i < PID_Qn; i++) {
		pid->error[i] = 0.0;
	}
#endif
	return 0;
}

int pid_setOutputConstrain(pid_controller_t *pid, double min, double max)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	pid->output_limit.min = min;
	pid->output_limit.max = max;
	return 0;
}

int pid_setSumConstrins(pid_controller_t *pid, double min, double max)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	pid->sum_limit.min = min;
	pid->sum_limit.max = max;
	return 0;
}

int pid_tuneParam(pid_controller_t *pid, double kp, double ki, uint32_t sample_time)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	if (sample_time == 0) {
		return -EINVAL;
	}
	if (ki == 0.0) {
		pid->set.sum = 0.0;
	}
	pid->set.sum = (ki == 0.0) ? 0.0 : pid->set.sum;
	pid->param.ki = ki;
	pid->param.kp = kp;
	pid->param.sample_time = sample_time;
	pid->param.sample_nominal = sample_time;
	if (pid->type == PID_TYPE_SPEED_UP) {
		pid_updateQ(pid);
	}
	return 0;
}

int pid_tuneDiffParam(pid_controller_t *pid, double kd, double kd2, double kd3)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	double kds[] = {kd, kd2, kd3};
	for (uint8_t i = 0; i < PID_KDn; i++) {
		pid->param.kd[i] = kds[i];
		pid->set.derivative[i] = 0;
	}
	if (pid->type == PID_TYPE_SPEED_UP) {
		pid_updateQ(pid);
	}
	return 0;
}

static int pid_classic(pid_controller_t *pid, double *output)
{
	double temp_p = pid->param.kp * pid->error[PID_ERROR0];

	pid->set.sum += pid->error[PID_ERROR0] * pid->param.sample_time;
	pid->set.sum = pid_checkWindup(pid, pid->set.sum);
	double temp_i = pid->param.ki * pid->set.sum;

	double temp_d = 0.0;
	for (uint8_t i = 0; (i < (PID_KDn - 1)) && (pid->param.kd[i] > 0.0); i++) {
		double diff0 = (pid->error[i] - pid->error[i + 1]) / pid->param.sample_time;
		double diff1 = (pid->error[i + 1] - pid->error[i + 2]) / pid->param.sample_time;
		double d0 = pid->param.kd[i] * diff0;
		double d1 = pid->param.kd[i + 1] * diff1;
		temp_d += (pid->alpha * d0 + (1 - pid->alpha) * d1);
	}
	*output = temp_p + temp_i + temp_d;
	return 0;
}

static int pid_speedUp(pid_controller_t *pid, double *output)
{
	double temp = pid->last_value;
	for (uint8_t i = 0; i < PID_Qn; i++) {
		temp += pid->param.q[i] * pid->error[i];
	}
    *output = temp;
	return 0;
}

static int pid_handleOldError(pid_controller_t *pid)
{
	if (pid == NULL) {
		return -1;
	}
	for (uint8_t i = PID_ERROR4; i > 0; i--) {
		pid->error[i] = pid->error[i - 1];
	}
	return 0;
}

static int pid_updateQ(pid_controller_t *pid)
{
	if (pid == NULL || pid->param.sample_time == 0) {
		return -1;
	}
	pid->param.q[PID_Q0] = pid->param.kp + pid->param.ki * pid->param.sample_time;
	pid->param.q[PID_Q0] += pid->param.kd[PID_KD1] / pid->param.sample_time;

	pid->param.q[PID_Q1] = -pid->param.kp - 2 * pid->param.kd[PID_KD1] / pid->param.sample_time;
	pid->param.q[PID_Q1] +=
		pid->param.kd[PID_KD2] / pid->param.sample_time / pid->param.sample_time;

	pid->param.q[PID_Q2] = pid->param.kd[PID_KD1] / pid->param.sample_time;
	pid->param.q[PID_Q2] +=
		pid->param.kd[PID_KD2] / pid->param.sample_time / pid->param.sample_time;
	pid->param.q[PID_Q2] += pid->param.kd[PID_KD3] / pid->param.sample_time /
				pid->param.sample_time / pid->param.sample_time;

	pid->param.q[PID_Q3] =
		pid->param.kd[PID_KD2] / pid->param.sample_time / pid->param.sample_time;
	pid->param.q[PID_Q3] += -2 * pid->param.kd[PID_KD3] / pid->param.sample_time /
				pid->param.sample_time / pid->param.sample_time;

	pid->param.q[PID_Q4] = pid->param.kd[PID_KD3] / pid->param.sample_time /
			       pid->param.sample_time / pid->param.sample_time;
	return 0;
}

static double pid_checkOvershoot(pid_constrain_t constrain, double value)
{
	if (constrain.min == constrain.max) {
		return value;
	}
	return CLAMP(value, constrain.min, constrain.max);
}

static double pid_checkWindup(pid_controller_t *pid, double input)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	return pid_checkOvershoot(pid->sum_limit, input);
}
