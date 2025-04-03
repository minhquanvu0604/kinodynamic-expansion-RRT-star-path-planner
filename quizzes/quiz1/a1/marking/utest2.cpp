#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../person.h"
#include "../processing.h"
using namespace std;



TEST (FunctionTest, CheckVacinated) {
    Person alice("Alice",50,false);
    EXPECT_EQ(alice.getVaccinatedStatus(), false);
    alice.vaccinate();
    EXPECT_EQ(alice.getVaccinatedStatus(), true);
}

TEST (FunctionTest, DetectOldest) {
    std::vector<Person> crowd;
    crowd.push_back(Person("Alice",32,true));
    crowd.push_back(Person("Bob",62,false));
    crowd.push_back(Person("Carol",72,true));
    crowd.push_back(Person("John",82,false));
    crowd.push_back(Person("Karen",42,true));

    std::vector<Person> oldest;
    oldest=oldestPerson(crowd);


    ASSERT_EQ(oldest.size(), 1);
    EXPECT_EQ(oldest.at(0).getName(), "John");


    /// What happens if 2 people have same age?
    crowd.push_back(Person("Michael",82,true));
    oldest=oldestPerson(crowd);

    ASSERT_EQ(oldest.size(), 2);
    EXPECT_EQ(oldest.at(0).getName(), "John");
    EXPECT_EQ(oldest.at(1).getName(), "Michael");


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

















//std::cout << "******************************" << std::endl;
//for (auto p : oldest){
//    std::cout << p.getName() << " " << p.getAge() << std::endl;
//}
