#ifndef CARINTERFACE_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CARINTERFACE_H

#include <string>

const double airDensity = 1.25; // air density (in kg/m³), at sea level has the value 1.25 kg/m³.
const double g = 9.81; //gravity  m/s2
const double power_conversion = 746.0; // converts from horse power HP to Watts
const double tyreFriction=1.5;

class CarInterface{
public:
    CarInterface(){};

  //! Gets the make
  /*!
    \return make of car
    \sa CarInterface() and setMake()
  */
  virtual std::string getMake(void) =0;

  //! Gets the model
  /*!
    \return model of car
    \sa CarInterface() and setModel()
  */
  virtual std::string getModel(void) = 0;
  //! Gets the current speed
  /*!
    \return current speed [m/s].
    \sa CarInterface() and accelerate() and decelerate();
  */
  virtual double getCurrentSpeed(void) =0;

  //! Gets the odometry
  /*!
    \return current odometry [m].
    \sa CarInterface() ;
  */
  virtual double getOdometry(void) = 0;

  //setters
  virtual void setMake(std::string) = 0;
  virtual void setModel(std::string) = 0;

  //! Computes the top speed
  /*!
    \return top speed [m/s].
    \sa CarInterface()
  */
  virtual double getTopSpeed(void) = 0;

  //! Get racing ID
  /*!
    \return get ID.
    \sa Car()
  */
  virtual unsigned int getID(void) = 0;

  /**
   This function decelerates the vehicle
   */
  virtual void decelerate(void) = 0;
  /**
   This function accelerates the vehicle
   */
  virtual void accelerate(void) = 0;

};


#endif // CARINTERFACE_H
