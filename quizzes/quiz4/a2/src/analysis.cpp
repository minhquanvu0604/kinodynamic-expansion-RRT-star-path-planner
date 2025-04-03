#include "analysis.h"
#include <cctype>
#include <iostream> // Only here for showing the code is working

namespace analysis {

    //! @todo
    //! TASK 4 - Refer to README.md and the Header file for full description
    unsigned int countCharacters(std::string sentence){
        unsigned int count = 0;
        count = sentence.size();
        return count;
    }

    //! @todo
    //! TASK 5 - Refer to README.md and the Header file for full description
    int getNumber(std::string sentence) {

        unsigned int num = 0;
        unsigned int i;
        
        for (i = 0; i < sentence.length(); i++) {
            if (isdigit(sentence[i])) break;
        }
        
        auto integer = sentence.substr(i, sentence.length() - i);
        num = atoi(integer.c_str());
        return num;
    }
}