#include <gtest/gtest.h>
#include "vegsec.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));

  // TODO: Run test on sample image

  return RUN_ALL_TESTS();
}
