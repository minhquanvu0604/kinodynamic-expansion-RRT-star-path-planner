#ifndef ACKERMAN_TEST_H
#define ACKERMAN_TEST_H

#include "gtest/gtest.h"
#include "linkcommand.h"

// The below is to setup our Unit Tests, Not required to understand, skip to wheer tests start
class AckermanTest : public testing::Test {
 protected:

  static void SetUpTestCase() {
    // Avoid reallocating static objects if called in subclasses of FooTest.
    if (pipesFakeOdo_ == nullptr) {
      pipesFakeOdo_ = new LinkCommand;
    }
  }


  static void TearDownTestCase() {
      if (pipesFakeOdo_ != nullptr){
        delete pipesFakeOdo_;
        pipesFakeOdo_ = nullptr;
      }
  }

  static LinkCommand* pipesFakeOdo_;

};

LinkCommand* AckermanTest::pipesFakeOdo_ = nullptr;


#endif // ACKERMAN_TEST_H
