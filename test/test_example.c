/*
 * Copyright (c) 2023 Diego Asanza <f.asanza@gmail.com>
 * Created on Mon May 29 2023
 */

/* SPDX-License-Identifier: BSD 3-Clause  */

#include <unity.h>

void setUp( void ) {

}

void tearDown( void ) {

}

void test_example( void ) 
{
    TEST_ASSERT_EQUAL(1, 1);
}

int main( void ) {
  UnityBegin("test/test_example.c");
  RUN_TEST(test_example, 20);
  return (UnityEnd());
}