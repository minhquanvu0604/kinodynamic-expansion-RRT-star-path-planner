#include "processing.h"
// You can add additional STL headers needed here, such as <algortithm>
#include <limits>

//3) TASK Implement function according to specification in header file
std::vector<Person> oldestPerson(std::vector<Person> crowd){
    static unsigned int maxage = 0;
    for (int i = 0; i < crowd.size(); i++)
        if ((crowd.at(i)).getAge() > maxage) maxage = (crowd.at(i)).getAge();

    for (int i = 0; i < crowd.size();)
        if ((crowd.at(i)).getAge() < maxage)
            crowd.erase(crowd.begin() + i);
        else i++;
    return crowd;
}


//4) TASK Implement function according to specification in header file
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff){
    for (int i = 0; i < crowd.size();) 
        if (((crowd.at(i)).getAge() >= ageCutoff) && !((crowd.at(i)).getVaccinatedStatus()))
            i++;
        else crowd.erase(crowd.begin() + i);
    return crowd;
}
