#ifndef CAR_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define CAR_H

#include <string>
#include "carinterface.h"
#include <chrono>         // std::chrono::seconds, time lapse etc

class Car : public CarInterface {
public:
  Car(std::string make, std::string model,unsigned int id, double height, double width,
      double horsePower, double dragCoefficient, double weight);

  //! Gets the make
  /*!
    \return make of car
    \sa Car() and setMake()
  */
  std::string getMake(void);

  //! Gets the model
  /*!
    \return model of car
    \sa Car() and setModel()
  */
  std::string getModel(void);
  //! Gets the current speed
  /*!
    \return current speed [m/s].
    \sa Car() and accelerate() and decelerate();
  */
  double getCurrentSpeed(void);

  //! Gets the odometry
  /*!
    \return current odometry [m].
    \sa Car() ;
  */
  double getOdometry(void);

  //setters
  void setMake(std::string);
  void setModel(std::string);

  //! Get racing ID
  /*!
    \return get ID.
    \sa Car()
  */
  unsigned int getID(void);


  //! Computes the top speed
  /*!
    \return top speed [m/s].
    \sa Car()
  */
  double getTopSpeed(void);


  /**
   This function decelerates the vehicle
   */
  void decelerate(void);
  /**
   This function accelerates the vehicle
   */
  void accelerate(void);

private:
  void updateOdometry(double dt);

protected:
  std::string make_;  //!< make of car
  std::string model_; //!< model of car
  double top_Speed_;  //!< top speed of car
  double currentSpeed_;//!< current speed of car
  unsigned int id_;//!< The id of car used for racing

  double area_;       //!< area in front of car
  double power_;      //!< power of car [W]
  //!< Drag coefficient ranges between 0.25 for the Honda Insight to 0.58 for the Jeep Wrangler TJ Soft Top.
  //!< The average value is 0.33
  double dragCoefficient_ ;
  double weight_;     //!< Weight of car [kg]

  bool carStationary=true;

  double odometry_;   //!< Distance travelled
  double time_;       //!< Current time
};


#endif // CAR_H
