#include "analysis.h"

#include <iostream> // Only here for showing the code is working
#include <thread>
#include <chrono>

Analysis::Analysis(std::shared_ptr<Radar> radarPtr):
    radarPtr_(radarPtr)
{

}

//! @todo
//! TASK 1 and 2 - Same implementation, just being called twice Refer to README.md and the Header file for full description
void Analysis::computeScanningSpeed(unsigned int samples, double& scanningSpeed){
  std::vector<double> data;  
  int scanTimes = -1;

  auto start = std::chrono::steady_clock::now();

  while (data.size() < samples){
    scanTimes++;
    std::vector<double> scanned = radarPtr_->getData();
    data.insert(data.end(), scanned.begin(), scanned.end());
  }

  auto end = std::chrono::steady_clock::now();
  
  std::chrono::duration<double> diff = end - start;
  double time = diff.count();

  scanningSpeed = time / scanTimes; 

  return;
}
