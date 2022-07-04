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

static void defex_rules_signature_check_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_rules_signature_check()); */
}


static void defex_public_key_verify_signature_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_public_key_verify_signature()); */
}


static void defex_keyring_init_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_keyring_init()); */
}


static void defex_keyring_alloc_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_keyring_alloc()); */
}


static void defex_calc_hash_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, defex_calc_hash()); */
}


static void blob_test(struct test *test)
{
    /* EXPECT_EQ(test, 1, blob()); */
}


static int defex_sign_test_init(struct test *test)
{
	/*
	 * test->priv = a_struct_pointer;
	 * if (!test->priv)
	 *    return -ENOMEM;
	 */

	return 0;
}

static void defex_sign_test_exit(struct test *test)
{
}

static struct test_case defex_sign_test_cases[] = {
	/* TEST FUNC DEFINES */
	TEST_CASE(defex_rules_signature_check_test),
	TEST_CASE(defex_public_key_verify_signature_test),
	TEST_CASE(defex_keyring_init_test),
	TEST_CASE(defex_keyring_alloc_test),
	TEST_CASE(defex_calc_hash_test),
	TEST_CASE(blob_test),
	{},
};

static struct test_module defex_sign_test_module = {
	.name = "defex_sign_test",
	.init = defex_sign_test_init,
	.exit = defex_sign_test_exit,
	.test_cases = defex_sign_test_cases,
};
module_test(defex_sign_test_module);

