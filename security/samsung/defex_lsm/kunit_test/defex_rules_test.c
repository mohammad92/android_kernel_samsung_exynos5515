/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <kunit/test.h>
#include <kunit/mock.h>

/* General test functions created by Generate_KUnit.sh */

static void static_rule_count_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, static_rule_count()); */
}


static int defex_rules_test_init(struct test *test)
{
	/*
	 * test->priv = a_struct_pointer;
	 * if (!test->priv)
	 *    return -ENOMEM;
	 */

	return 0;
}

static void defex_rules_test_exit(struct test *test)
{
}

static struct test_case defex_rules_test_cases[] = {
	/* TEST FUNC DEFINES */
	TEST_CASE(static_rule_count_test),
	{},
};

static struct test_module defex_rules_test_module = {
	.name = "defex_rules_test",
	.init = defex_rules_test_init,
	.exit = defex_rules_test_exit,
	.test_cases = defex_rules_test_cases,
};
module_test(defex_rules_test_module);

