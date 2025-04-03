#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../analysis.h"
using namespace std;


TEST (RadarTest, MinRangeTiming) {
    std::shared_ptr<Radar> radarPtr(new Radar());
    radarPtr->setMaxDistance(80.0);
    radarPtr->start();

    //! Created a pointer to Analsyis (as we need to pass this pointer as 2nd parameter to thread
    std::shared_ptr<Analysis> analysisPtr(new Analysis(radarPtr));

    double scanningSpeed=0;
    unsigned int samples=100;

    //! Create thread
    std::thread timing_thread(&Analysis::computeScanningSpeed,analysisPtr,samples,std::ref(scanningSpeed));

    timing_thread.join();

    ASSERT_NEAR(scanningSpeed,0.1,1e-2);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
