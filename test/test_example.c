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