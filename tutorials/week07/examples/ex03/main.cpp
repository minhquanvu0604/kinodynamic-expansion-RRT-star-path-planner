#include <iostream>
#include <thread>
#include <vector>

#include "radar.h"
#include "dataprocessing.h"

int main (void){

  //! We create a vector of pointers to Radar
  std::vector<Radar*> radars;
  //! Push back 3 radars
  radars.push_back(new Radar);
  radars.push_back(new Radar);
  radars.push_back(new Radar);

  //! Start thread of each radar
  for (auto radar : radars){
    radar->start();
  }

  //! Created a pointer to data processing
  std::shared_ptr<DataProcessing> dataProcessingPtr(new DataProcessing());
  //! Create thread
  // THIS THREAD RUNS CONTINUOUSLY IN THE BACKGROUND
   std::thread processing_thread(&DataProcessing::findClosestReading,dataProcessingPtr);

   std::this_thread::sleep_for (std::chrono::milliseconds(1000));
   //! Pass the radars
   dataProcessingPtr->setRadars(radars);


  //! Join thread when we have finished
  processing_thread.join();

  //! Call destructors for each radar
  for (auto radar : radars){
    delete radar;
  }


  return 0;
}

