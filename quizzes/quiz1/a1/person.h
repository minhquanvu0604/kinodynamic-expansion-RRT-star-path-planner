#ifndef PERSON_H  // An 'include guard' to prevent double declaration of any identifiers in this library
#define PERSON_H

#include <string>

class Person {
public:
    //    1) TASK The `Person` class is missing a special member function. This function needs to enable crteating an object of
    //    `Person` class with the `name` , `age` and `vacinated` initialised with values supplied by user of the class.
    //    You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as
    //    well as implement this function in the [implementation file of Person class](./a1/person.cpp).
  Person(std::string name, unsigned int age, bool vaccinated);

  /**
   * @brief Function that obtains name
   * @return name of person
   */
  std::string getName(void);

  /**
   * @brief Function that retruns the age
   * @return age in years
   */
  unsigned int getAge(void);

  /**
   * @brief Function that retruns vaccinated status
   * @return vaccinated status
   */
  bool getVaccinatedStatus(void);

  /**
   * @brief Increments age in years (birthday)
   */
  void birthday(void);

  /**
   * @brief Function that vaccinates a person if they have not previously been vaccinated
   * @return indicates if a person should have been vaccinated (true is they have not already been vacinated)
   */
  bool vaccinate();

private:
  std::string name_; //!< name of person
  unsigned int age_; //!< age in years
  bool vaccinated_;  //!< vaccinated status (true - has been given vaccine, false - not vaccinated)
};


#endif // PERSON_H
