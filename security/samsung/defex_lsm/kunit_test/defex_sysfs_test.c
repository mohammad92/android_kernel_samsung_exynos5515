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

static void rules_lookup2_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, rules_lookup2()); */
}


static void rules_lookup_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, rules_lookup()); */
}


static void parse_static_rules_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, parse_static_rules()); */
}


static void lookup_tree_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, lookup_tree()); */
}


static void lookup_dir_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, lookup_dir()); */
}


static void load_rules_late_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, load_rules_late()); */
}


static void do_load_rules_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, do_load_rules()); */
}


static void defex_load_rules_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_load_rules()); */
}


static void defex_integrity_default_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_integrity_default()); */
}


static void defex_init_sysfs_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_init_sysfs()); */
}


static void defex_check_integrity_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_check_integrity()); */
}


static void check_system_mount_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, check_system_mount()); */
}


static void check_rules_ready_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, check_rules_ready()); */
}


static void bootmode_setup_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, bootmode_setup()); */
}


static int defex_sysfs_test_init(struct test *test)
{
	/*
	 * test->priv = a_struct_pointer;
	 * if (!test->priv)
	 *    return -ENOMEM;
	 */

	return 0;
}

static void defex_sysfs_test_exit(struct test *test)
{
}

static struct test_case defex_sysfs_test_cases[] = {
	/* TEST FUNC DEFINES */
	TEST_CASE(rules_lookup2_test),
	TEST_CASE(rules_lookup_test),
	TEST_CASE(parse_static_rules_test),
	TEST_CASE(lookup_tree_test),
	TEST_CASE(lookup_dir_test),
	TEST_CASE(load_rules_late_test),
	TEST_CASE(do_load_rules_test),
	TEST_CASE(defex_load_rules_test),
	TEST_CASE(defex_integrity_default_test),
	TEST_CASE(defex_init_sysfs_test),
	TEST_CASE(defex_check_integrity_test),
	TEST_CASE(check_system_mount_test),
	TEST_CASE(check_rules_ready_test),
	TEST_CASE(bootmode_setup_test),
	{},
};

static struct test_module defex_sysfs_test_module = {
	.name = "defex_sysfs_test",
	.init = defex_sysfs_test_init,
	.exit = defex_sysfs_test_exit,
	.test_cases = defex_sysfs_test_cases,
};
module_test(defex_sysfs_test_module);

