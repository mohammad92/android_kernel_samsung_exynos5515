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

static void defex_file_cache_update_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_file_cache_update()); */
}


static void defex_file_cache_init_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_file_cache_init()); */
}


static void defex_file_cache_find_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_file_cache_find()); */
}


static void defex_file_cache_delete_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_file_cache_delete()); */
}


static void defex_file_cache_add_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_file_cache_add()); */
}


static void defex_caches_lock_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_caches_lock()); */
}


static int defex_caches_test_init(struct test *test)
{
	/*
	 * test->priv = a_struct_pointer;
	 * if (!test->priv)
	 *    return -ENOMEM;
	 */

	return 0;
}

static void defex_caches_test_exit(struct test *test)
{
}

static struct test_case defex_caches_test_cases[] = {
	/* TEST FUNC DEFINES */
	TEST_CASE(defex_file_cache_update_test),
	TEST_CASE(defex_file_cache_init_test),
	TEST_CASE(defex_file_cache_find_test),
	TEST_CASE(defex_file_cache_delete_test),
	TEST_CASE(defex_file_cache_add_test),
	TEST_CASE(defex_caches_lock_test),
	{},
};

static struct test_module defex_caches_test_module = {
	.name = "defex_caches_test",
	.init = defex_caches_test_init,
	.exit = defex_caches_test_exit,
	.test_cases = defex_caches_test_cases,
};
module_test(defex_caches_test_module);

