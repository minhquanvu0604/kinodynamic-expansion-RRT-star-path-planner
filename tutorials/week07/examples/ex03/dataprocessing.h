#ifndef DATAPROCESSING_H
#define DATAPROCESSING_H

#include <vector>
#include "radar.h"

class DataProcessing
{
public:
  DataProcessing();
  void setRadars(std::vector<Radar*> radars);
  void findClosestReading();

private:
  std::vector<Radar*> radars_;

};

#endif // DATAPROCESSING_H
