#ifndef ANALYSIS_H
#define ANALYSIS_H


#include <vector>
#include <thread>
#include "radar.h"

class Analysis
{
public:
  Analysis(std::shared_ptr<Radar> radarPtr);
  /**
   * @brief computes the scanning speed by taking a number of samples (as requested by user) and taking the average
   * of the time for aquiring the samples.
   * @param number of samples to obtai from radar in order to determine scanning speed
   * @return scaningSpeed the actual scanning speed
   */
  void computeScanningSpeed(unsigned int samples, double& scanningSpeed);

private:
  std::shared_ptr<Radar> radarPtr_;//!< A shared pointer to the Radar

};

#endif // ANALYSIS_H
