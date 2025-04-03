#include <iostream>
#include <thread>
#include "analysis.h"

using namespace std;



int main ()
{

    Analysis analysis;

    while(1){
      std::this_thread::sleep_for (std::chrono::milliseconds(500));
    }

    return 0;
}



