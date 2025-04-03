#ifndef MOCKCAR_H
#define MOCKCAR_H

#include "car.h"

/*!
 *  \ingroup   ac_mock Mock Car
 *  \brief     Mock Car Class
 *  \details
 *  This class is used to inject readings, and override the odometry of Car Class.\n
 *  It inherits from Car, however can store mock car data to test the class.\n
 *  \author    Alen Alempijevic
 *  \version   1.00-2
 *  \date      2021-09-10
 *  \pre       none
 *  \bug       none reported as of 2021-09-10
 *  \warning
 */

class MockCar: public Car
{
public:
  MockCar(std::string make, std::string model,unsigned int id, double height, double width,
          double horsePower, double dragCoefficient, double weight, double mockOdo);

  double getOdometry();

protected:
  double mockOdo_;
};

#endif // MOCKCAR_H
