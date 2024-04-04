#ifndef SKIDSTEER_TEST_H
#define SKIDSTEER_TEST_H

#include "gtest/gtest.h"
#include "pfmshog.h"

// The below is to setup our Unit Tests, Not required to understand, skip to wheer tests start
class SkidSteerTest : public testing::Test {
 protected:

  static void SetUpTestCase() {
    // Avoid reallocating static objects if called in subclasses of FooTest.
    if (pfmsHogPtr_ == nullptr) {
      pfmsHogPtr_ = new PfmsHog(pfms::PlatformType::SKIDSTEER);
    }
  }


  static void TearDownTestCase() {
      if (pfmsHogPtr_ != nullptr){
        delete pfmsHogPtr_;
        pfmsHogPtr_ = nullptr;
      }
  }

  static PfmsHog* pfmsHogPtr_;

};

PfmsHog* SkidSteerTest::pfmsHogPtr_ = nullptr;


#endif // ACKERMAN_TEST_H
