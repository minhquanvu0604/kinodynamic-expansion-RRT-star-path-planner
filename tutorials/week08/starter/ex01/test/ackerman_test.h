#ifndef ACKERMAN_TEST_H
#define ACKERMAN_TEST_H

#include "gtest/gtest.h"
#include "linkcommand.h"

// The below is to setup our Unit Tests, Not required to understand, skip to where tests start
class AckermanTest : public testing::Test {
 protected:

  static void SetUpTestCase() {
    // Avoid reallocating static objects if called in subclasses of FooTest.
    if (linkCommand_ == nullptr) {
      linkCommand_ = new LinkCommand;
    }
  }


  static void TearDownTestCase() {
      if (linkCommand_ != nullptr){
        delete linkCommand_;
        linkCommand_ = nullptr;
      }
  }

  static LinkCommand* linkCommand_;

};

LinkCommand* AckermanTest::linkCommand_ = nullptr;


#endif // ACKERMAN_TEST_H
