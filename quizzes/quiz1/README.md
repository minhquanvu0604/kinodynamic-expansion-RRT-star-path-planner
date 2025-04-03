Quiz 1
======

Part A1
------

1) TASK
The `Person` class is missing a special member function. This function needs to enable creating an object of `Person` class with the `name` , `age` and `vaccinated` status initialised with values supplied by user of the class. You will need to add the declaration of this member function in the [header of Person class](./a1/person.h) as well as implement this function in [implementation file of Person class](./a1/person.cpp).

2) TASK
Implement the method `vaccinate` in `Person` class. This function returns a `bool` indicating if a person can be given a vaccine. The person can only be given a vaccine, if they are not already vaccinated (so the function returns a bool if we try to vaccinate a person that is already vaccinated).  When a person is given a vaccine, their `vaccinated` status should change. 

3) TASK
Implement [oldestPerson](./a1/processing.h) function that returns the `Person` who is the oldest person(s) of the `crowd`. There could be multiple people of same age, therefore the return type is a vector.

4) TASK
Implement [eligibleForVaccine](./a1/processing.h) function that returns the `people` from the `crowd` that need to be vaccinated. Criteria is that they are older or equal to the specified `ageCuttoff` and have not been previously vaccinated. The age cut-off is inclusive of the age specified (an example is 18+, which is 18 and above).

Part A2
------

1. TASKS

* Modify the `Rectangle` class so it inherits from the base class of shape [shape](./a2/shape.h). 

* Correct the missing access specifiers of base class [shape](./b2/shape.h) so that the implementation of the [`Rectangle` constructor](./a2/rectangle.cpp) can still access the required member variable of `Shape` but have it restricted to users of a `Rectangle` object.

* Enable the  `Rectangle` class to have another special member function that enables the `Rectangle` to on creation have `width` , `height`  initialised with values supplied by user of the class. This function needs to set the description on creation of the class to be either `square` or `rectangle` depending on the `width` and  `heigth` supplied by the user. 

  **NOTE: The function can not take a description, only the width and height **

2. TASK

* Implement function `void removeLargerThanArea(std::vector<Shape*> &shapes, double limit);` which is in [processing.h](./a2/processing.h). The function removes from the container of `shapes` any shape that has an area greater than the `limit`.  

In your main you could create a number of rectangles and check the function works as specified. 

