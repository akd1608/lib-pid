/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Copyright (c) 2026 akd (akd1608@mailbox.org)
 */
#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <pid_controller.h>

static pid_controller_t pid;

/* A second run in the same millisecond does nothing, so let time pass first. */
static double step(double input)
{
	double out = -12345.0;

	k_sleep(K_MSEC(1));
	zassert_ok(pid_run(&pid, input, &out));
	return out;
}

/* zassert_within() evaluates its arguments more than once */
#define zassert_step(input, expected, delta)        \
	do {                                        \
		double got_ = step(input);          \
		zassert_within(got_, expected, delta); \
	} while (0)

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	pid_init(&pid, PID_TYPE_CLASSIC);
}

ZTEST_SUITE(pid, NULL, NULL, before, NULL, NULL);

ZTEST(pid, test_null_is_rejected)
{
	double out;

	zassert_equal(pid_init(NULL, PID_TYPE_CLASSIC), -ENODEV);
	zassert_equal(pid_reset(NULL), -ENODEV);
	zassert_equal(pid_run(NULL, 0.0, &out), -ENODEV);
	zassert_equal(pid_run(&pid, 0.0, NULL), -EINVAL);
	zassert_equal(pid_setSetPoint(NULL, 0.0), -ENODEV);
	zassert_equal(pid_setOutputConstrain(NULL, 0.0, 1.0), -ENODEV);
	zassert_equal(pid_setSumConstrins(NULL, 0.0, 1.0), -ENODEV);
	zassert_equal(pid_tuneParam(NULL, 1.0, 0.0, 100U), -ENODEV);
	zassert_equal(pid_tuneDiffParam(NULL, 0.0, 0.0, 0.0), -ENODEV);
}

ZTEST(pid, test_sample_time_zero_is_rejected)
{
	zassert_equal(pid_tuneParam(&pid, 1.0, 0.0, 0U), -EINVAL);
}

ZTEST(pid, test_init_clears_everything)
{
	memset(&pid, 0x5a, sizeof(pid));
	zassert_ok(pid_init(&pid, PID_TYPE_CLASSIC));
	pid_tuneParam(&pid, 2.0, 0.0, 100U);
	pid_setSetPoint(&pid, 10.0);
	zassert_step(4.0, 12.0, 1e-9);
}

ZTEST(pid, test_reset_keeps_gains_and_limits)
{
	pid_tuneParam(&pid, 1.0, 1.0, 100U);
	pid_setOutputConstrain(&pid, -1000.0, 1000.0);
	pid_setSetPoint(&pid, 1.0);
	step(0.0);
	step(0.0);
	zassert_true(pid.set.sum > 0.0);

	zassert_ok(pid_reset(&pid));
	zassert_within(pid.set.sum, 0.0, 1e-12);
	zassert_within(pid.last_value, 0.0, 1e-12);
	zassert_within(pid.error[PID_ERROR1], 0.0, 1e-12);
	zassert_within(pid.param.kp, 1.0, 1e-12);
	zassert_true(pid.output_limit.active);
}

ZTEST(pid, test_classic_proportional)
{
	pid_tuneParam(&pid, 2.0, 0.0, 100U);
	pid_setSetPoint(&pid, 10.0);
	zassert_step(4.0, 12.0, 1e-9);
}

ZTEST(pid, test_output_limit)
{
	pid_tuneParam(&pid, 100.0, 0.0, 100U);
	pid_setOutputConstrain(&pid, -5.0, 5.0);
	pid_setSetPoint(&pid, 10.0);

	zassert_step(0.0, 5.0, 1e-9);
	zassert_step(20.0, -5.0, 1e-9);
}

ZTEST(pid, test_no_limit_without_a_call)
{
	pid_tuneParam(&pid, 100.0, 0.0, 100U);
	pid_setSetPoint(&pid, 10.0);
	zassert_step(0.0, 1000.0, 1e-9);
}

ZTEST(pid, test_equal_limits_hold_the_value)
{
	pid_tuneParam(&pid, 100.0, 0.0, 100U);
	zassert_ok(pid_setOutputConstrain(&pid, 0.0, 0.0));
	pid_setSetPoint(&pid, 10.0);
	zassert_step(0.0, 0.0, 1e-12);
}

ZTEST(pid, test_reversed_limits_are_rejected)
{
	zassert_equal(pid_setOutputConstrain(&pid, 1.0, 0.0), -EINVAL);
	zassert_equal(pid_setSumConstrins(&pid, 1.0, 0.0), -EINVAL);
	zassert_false(pid.output_limit.active);
}

ZTEST(pid, test_integral_adds_up_over_real_time)
{
	double out = 0.0;

	pid_tuneParam(&pid, 0.0, 1.0, 100U);
	pid_setSumConstrins(&pid, -1.0e6, 1.0e6);
	pid_setSetPoint(&pid, 2.0);

	/* first run: one regular step */
	k_sleep(K_MSEC(1));
	pid_run(&pid, 0.0, &out);
	zassert_within(pid.set.sum, 200.0, 1e-9);

	k_sleep(K_MSEC(100));
	pid_run(&pid, 0.0, &out);
	zassert_within(pid.param.sample_time, 100U, 20U);
	zassert_within(pid.set.sum, 400.0, 40.0);

	k_sleep(K_MSEC(50));
	pid_run(&pid, 0.0, &out);
	zassert_within(pid.param.sample_time, 50U, 20U);
	zassert_within(pid.set.sum, 500.0, 60.0);
	zassert_within(out, pid.set.sum, 1e-9);
}

ZTEST(pid, test_integral_stops_at_limit_and_runs_back)
{
	pid_tuneParam(&pid, 0.0, 1.0, 100U);
	pid_setSumConstrins(&pid, -300.0, 300.0);
	pid_setSetPoint(&pid, 2.0);

	for (int i = 0; i < 5; i++) {
		step(0.0);
		k_sleep(K_MSEC(100));
	}
	zassert_within(pid.set.sum, 300.0, 1e-9);

	/* negative error takes it back down at once, no wound-up reserve */
	step(3.0);
	zassert_true(pid.set.sum < 300.0);
	zassert_true(pid.set.sum > 150.0);
}

ZTEST(pid, test_pause_counts_as_one_step)
{
	pid_tuneParam(&pid, 0.0, 1.0, 10U);
	pid_setSumConstrins(&pid, -1.0e6, 1.0e6);
	pid_setSetPoint(&pid, 1.0);

	step(0.0);
	double before_pause = pid.set.sum;

	/* 100 regular steps without a run, the default limit is 10 */
	k_sleep(K_MSEC(1000));
	step(0.0);
	zassert_equal(pid.param.sample_time, 10U);
	zassert_within(pid.set.sum - before_pause, 10.0, 1e-9);
}

ZTEST(pid, test_second_run_in_same_millisecond_returns_last_value)
{
	double a = 0.0;
	double b = 0.0;

	pid_tuneParam(&pid, 1.0, 0.0, 100U);
	pid_tuneDiffParam(&pid, 1.0, 0.0, 0.0);
	pid_setSetPoint(&pid, 1.0);

	k_sleep(K_MSEC(10));
	pid_run(&pid, 0.0, &a);
	pid_run(&pid, 0.5, &b);
	zassert_within(b, a, 1e-12);
	zassert_false(isnan(b) || isinf(b));
}

ZTEST(pid, test_classic_derivative_works_with_kd_alone)
{
	pid_tuneParam(&pid, 0.0, 0.0, 10U);
	pid_tuneDiffParam(&pid, 100.0, 0.0, 0.0);
	pid_setSetPoint(&pid, 0.0);

	step(0.0);

	/* error goes from 0 to -5: kd * change / measured time */
	double out = step(5.0);

	zassert_within(out, 100.0 * -5.0 / pid.param.sample_time, 1e-9);
	zassert_true(out < 0.0);
}

ZTEST(pid, test_speed_up_holds_at_constant_error)
{
	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 1.0, 0.0, 100U);
	pid_setSetPoint(&pid, 5.0);

	/* P only in velocity form: first step takes the error, then it holds. */
	zassert_step(0.0, 5.0, 1e-9);
	zassert_step(0.0, 5.0, 1e-9);
}

ZTEST(pid, test_speed_up_derivative_terms_add_nothing_at_constant_error)
{
	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 0.0, 0.0, 10U);
	pid_tuneDiffParam(&pid, 3.0, 40.0, 500.0);
	pid_setSetPoint(&pid, 5.0);

	double sum = 0.0;

	for (int i = 0; i < PID_Qn; i++) {
		sum += pid.param.q[i];
	}
	zassert_within(sum, 0.0, 1e-9);

	/* after the error history has filled, the output stands still */
	double out = 0.0;

	for (int i = 0; i < 8; i++) {
		out = step(0.0);
	}
	zassert_step(0.0, out, 1e-9);
}

ZTEST(pid, test_speed_up_tuning_after_run_keeps_the_gain)
{
	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 2.0, 0.0, 100U);
	pid_tuneDiffParam(&pid, 10.0, 0.0, 0.0);
	double q0 = pid.param.q[PID_Q0];

	pid_setSetPoint(&pid, 1.0);
	step(0.0);
	k_sleep(K_MSEC(5));
	step(0.0);
	zassert_true(pid.param.sample_time < 100U);

	pid_tuneDiffParam(&pid, 10.0, 0.0, 0.0);
	zassert_within(pid.param.q[PID_Q0], q0, 1e-12);
}

ZTEST(pid, test_speed_up_limit_does_not_keep_the_output_high)
{
	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 4.0, 0.00005, 250U);
	pid_tuneDiffParam(&pid, 20000.0, 0.0, 0.0);
	pid_setOutputConstrain(&pid, 0.0, 100.0);
	pid_setSetPoint(&pid, 100.0);

	/* long saturation well below the setpoint */
	for (int i = 0; i < 200; i++) {
		zassert_step(0.0, 100.0, 100.0);
	}
	/* then 15 K above it: the output must go to 0, with the bug it stayed at 92 */
	double out = 100.0;

	for (int i = 0; i < 40; i++) {
		out = step(115.0);
	}
	zassert_within(out, 0.0, 1e-9);

	/* and comes back when the error turns around */
	for (int i = 0; i < 10; i++) {
		out = step(90.0);
	}
	zassert_true(out > 30.0, "%f", out);
}

ZTEST(pid, test_speed_up_integral_holds_at_the_limit)
{
	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 0.0, 1.0, 10U);
	pid_setOutputConstrain(&pid, 0.0, 100.0);
	pid_setSetPoint(&pid, 1.0);

	for (int i = 0; i < 50; i++) {
		step(0.0);
	}
	/* I only: raw stops at the limit instead of winding up */
	zassert_true(pid.raw_value <= 110.0, "%f", pid.raw_value);

	/* so the first negative error is felt at once */
	double got = step(2.0);

	zassert_true(got < 100.0);
}
