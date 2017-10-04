/**
 * @file main.cpp
 * @brief main method for Tests
 * Runs All Tests and returns 0 if successful
 * @return 0
 *
 * @author Vaibav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 */

#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
