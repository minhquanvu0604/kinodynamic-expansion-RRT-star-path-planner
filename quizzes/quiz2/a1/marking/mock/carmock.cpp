#include "carmock.h"

MockCar::MockCar(std::string make, std::string model,unsigned int id, double height, double width,
                 double horsePower, double dragCoefficient, double weigth, double mockOdo):
    Car(make, model, id, height, width,horsePower, dragCoefficient, weigth) {
  //Extra constructor for easy mocking
  mockOdo_ = mockOdo;
}


double MockCar::getOdometry() {
  return mockOdo_;
}
