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
#include <string.h>

static int pid_updateQ(pid_controller_t *pid);
static int pid_classic(pid_controller_t *pid, double *output);
static int pid_speedUp(pid_controller_t *pid, double *output);
static int pid_handleOldError(pid_controller_t *pid);
static double pid_checkOvershoot(pid_constrain_t constrain, double value);

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
	return elapsed > 0 ? (uint32_t)elapsed : 1U;
}

int pid_init(pid_controller_t *pid, pid_type_t type)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	memset(pid, 0, sizeof(*pid));
	pid->type = type;
	pid->alpha = CONFIG_PID_FILTER_COEFFICIENT_ALPHA / 1000.f;
	return 0;
}

int pid_reset(pid_controller_t *pid)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	memset(&pid->set, 0, sizeof(pid->set));
	memset(pid->error, 0, sizeof(pid->error));
	pid->last_value = 0.0;
	pid->raw_value = 0.0;
	pid->last_stamp = 0U;
	return 0;
}

int pid_run(pid_controller_t *pid, double input, double *output)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	if (output == NULL) {
		return -EINVAL;
	}

	int64_t now = k_uptime_get();

	/* Second run in the same millisecond: no time passed, nothing to do */
	if (pid->last_stamp != 0U && now == (int64_t)pid->last_stamp) {
		*output = pid->last_value;
		return 0;
	}

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
	if (pid == NULL) {
		return -ENODEV;
	}
	pid->set_point = set_point;
#if CONFIG_PID_CONTROLLER_FLUSH_BY_NEW_SETPOINT
	pid_reset(pid);
#endif
	return 0;
}

int pid_setOutputConstrain(pid_controller_t *pid, double min, double max)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	if (min > max) {
		return -EINVAL;
	}
	pid->output_limit.min = min;
	pid->output_limit.max = max;
	pid->output_limit.active = true;
	return 0;
}

int pid_setSumConstrins(pid_controller_t *pid, double min, double max)
{
	if (pid == NULL) {
		return -ENODEV;
	}
	if (min > max) {
		return -EINVAL;
	}
	pid->sum_limit.min = min;
	pid->sum_limit.max = max;
	pid->sum_limit.active = true;
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
	pid->set.sum = pid_checkOvershoot(pid->sum_limit, pid->set.sum);
	double temp_i = pid->param.ki * pid->set.sum;

	double temp_d = 0.0;
	for (uint8_t i = 0; (i < (PID_KDn - 1)) && (pid->param.kd[i] > 0.0); i++) {
		double diff0 = (pid->error[i] - pid->error[i + 1]) / pid->param.sample_time;
		double diff1 = (pid->error[i + 1] - pid->error[i + 2]) / pid->param.sample_time;
		double d0 = pid->param.kd[i] * diff0;
		double d1 = pid->param.kd[i] * diff1;
		temp_d += (pid->alpha * d0 + (1 - pid->alpha) * d1);
	}
	*output = temp_p + temp_i + temp_d;
	return 0;
}

static int pid_speedUp(pid_controller_t *pid, double *output)
{
	double step = 0.0;

	for (uint8_t i = 0; i < PID_Qn; i++) {
		step += pid->param.q[i] * pid->error[i];
	}

	/*
	 * Run on the value before the limit: a cut-off step must not be lost, or
	 * only its way back arrives and the output stays wrong.  While that value
	 * is past a limit, the integral part stops pushing further out.
	 */
	double step_i = pid->param.ki * pid->param.sample_nominal * pid->error[PID_ERROR0];
	double raw = pid->raw_value + step - step_i;
	const pid_constrain_t *lim = &pid->output_limit;
	bool hold = lim->active && ((raw >= lim->max && step_i > 0.0) ||
				    (raw <= lim->min && step_i < 0.0));

	if (!hold) {
		raw += step_i;
	}
	pid->raw_value = raw;
	*output = raw;
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

/*
 * Velocity form, T is the regular step from tuning, not the measured time.
 * Every kd term is a second difference, so a constant error adds nothing:
 *   kd1/T   * (e0 - 2 e1 + e2)
 *   kd2/T^2 * (e1 - 2 e2 + e3)
 *   kd3/T^3 * (e2 - 2 e3 + e4)
 */
static int pid_updateQ(pid_controller_t *pid)
{
	if (pid == NULL || pid->param.sample_nominal == 0U) {
		return -1;
	}

	const double t = pid->param.sample_nominal;
	const double kd1 = pid->param.kd[PID_KD1] / t;
	const double kd2 = pid->param.kd[PID_KD2] / t / t;
	const double kd3 = pid->param.kd[PID_KD3] / t / t / t;

	pid->param.q[PID_Q0] = pid->param.kp + pid->param.ki * t + kd1;
	pid->param.q[PID_Q1] = -pid->param.kp - 2.0 * kd1 + kd2;
	pid->param.q[PID_Q2] = kd1 - 2.0 * kd2 + kd3;
	pid->param.q[PID_Q3] = kd2 - 2.0 * kd3;
	pid->param.q[PID_Q4] = kd3;
	return 0;
}

static double pid_checkOvershoot(pid_constrain_t constrain, double value)
{
	if (!constrain.active) {
		return value;
	}
	return CLAMP(value, constrain.min, constrain.max);
}
