#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../person.h"
#include "../processing.h"
using namespace std;


TEST (ClassTest, CreateObject) {
    Person alice("Alice",50,false);
    EXPECT_EQ(alice.getAge(), 50);
}

TEST (FunctionTest, CheckVacinated) {
    Person kubo("Kubo",18,false);
    EXPECT_EQ(kubo.getVaccinatedStatus(), false);
    EXPECT_EQ(kubo.vaccinate(), true);
}

TEST (FunctionTest, CheckAge_1) {
    std::vector<Person> waifu;
    waifu.push_back(Person("Mikasa",30,true));
    waifu.push_back(Person("Shikimori",20,false));
    waifu.push_back(Person("Shiina",17,true));
    waifu.push_back(Person("AI",15,false));
    waifu.push_back(Person("Hori",22,false));
    waifu.push_back(Person("Yuri",16,false));
    waifu.push_back(Person("Sawako",24,false));

    unsigned int age = 18;

    std::vector<Person> needVaccine;
    needVaccine = eligibleForVaccine(waifu,age);

    ASSERT_EQ(needVaccine.size(),3);
    EXPECT_EQ(needVaccine.at(0).getName(), "Shikimori");
    EXPECT_EQ(needVaccine.at(1).getName(), "Hori");
    EXPECT_EQ(needVaccine.at(2).getName(), "Sawako");

    EXPECT_EQ(needVaccine.at(0).getAge(), 20);
    EXPECT_EQ(needVaccine.at(1).getAge(), 22);
    EXPECT_EQ(needVaccine.at(2).getAge(), 24);
}

TEST (FuctionTest2, CheckAge_2){
  std::vector<Person>waifu;
  waifu.push_back(Person("Mikasa",20,true));
  waifu.push_back(Person("Shikimori",19,true));
  waifu.push_back(Person("Shiina",18,true));
  waifu.push_back(Person("Sakura",17,true));
  waifu.push_back(Person("Kubo",20,true));

  std::vector<Person> checkOldest;
  checkOldest=oldestPerson(waifu);

  EXPECT_EQ(checkOldest.size(), 2);
  EXPECT_EQ(checkOldest.at(0).getName(), "Mikasa");
  EXPECT_EQ(checkOldest.at(1).getName(), "Kubo");

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
