/*
 * SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 * Copyright (c) 2026 akd (akd1608@mailbox.org)
 */
#include <errno.h>
#include <string.h>
#include <zephyr/ztest.h>
#include <pid_controller.h>

static pid_controller_t pid;

static void before(void *fixture)
{
	ARG_UNUSED(fixture);
	memset(&pid, 0, sizeof(pid));
}

ZTEST_SUITE(pid, NULL, NULL, before, NULL, NULL);

ZTEST(pid, test_null_is_rejected)
{
	zassert_equal(pid_setOutputConstrain(NULL, 0.0, 1.0), -ENODEV);
	zassert_equal(pid_setSumConstrins(NULL, 0.0, 1.0), -ENODEV);
	zassert_equal(pid_tuneParam(NULL, 1.0, 0.0, 100U), -ENODEV);
	zassert_equal(pid_tuneDiffParam(NULL, 0.0, 0.0, 0.0), -ENODEV);
}

ZTEST(pid, test_sample_time_zero_is_rejected)
{
	zassert_equal(pid_tuneParam(&pid, 1.0, 0.0, 0U), -EINVAL);
}

ZTEST(pid, test_classic_proportional)
{
	double out = 0.0;

	pid_init(&pid, PID_TYPE_CLASSIC);
	zassert_ok(pid_tuneParam(&pid, 2.0, 0.0, 100U));
	zassert_ok(pid_setSetPoint(&pid, 10.0));
	zassert_ok(pid_run(&pid, 4.0, &out));
	zassert_within(out, 12.0, 1e-9);
}

ZTEST(pid, test_output_limit)
{
	double out = 0.0;

	pid_init(&pid, PID_TYPE_CLASSIC);
	pid_tuneParam(&pid, 100.0, 0.0, 100U);
	pid_setOutputConstrain(&pid, -5.0, 5.0);
	pid_setSetPoint(&pid, 10.0);

	pid_run(&pid, 0.0, &out);
	zassert_within(out, 5.0, 1e-9);
	pid_run(&pid, 20.0, &out);
	zassert_within(out, -5.0, 1e-9);
}

ZTEST(pid, test_equal_limits_mean_no_limit)
{
	double out = 0.0;

	pid_init(&pid, PID_TYPE_CLASSIC);
	pid_tuneParam(&pid, 100.0, 0.0, 100U);
	pid_setSetPoint(&pid, 10.0);
	pid_run(&pid, 0.0, &out);
	zassert_within(out, 1000.0, 1e-9);
}

ZTEST(pid, test_speed_up_holds_at_constant_error)
{
	double out = 0.0;

	pid_init(&pid, PID_TYPE_SPEED_UP);
	pid_tuneParam(&pid, 1.0, 0.0, 100U);
	pid_setSetPoint(&pid, 5.0);

	/* P only in velocity form: first step takes the error, then it holds. */
	pid_run(&pid, 0.0, &out);
	zassert_within(out, 5.0, 1e-9);
	pid_run(&pid, 0.0, &out);
	zassert_within(out, 5.0, 1e-9);
}

ZTEST(pid, test_ki_zero_clears_sum)
{
	pid.set.sum = 3.0;
	pid_tuneParam(&pid, 1.0, 0.0, 100U);
	zassert_within(pid.set.sum, 0.0, 1e-12);
}
