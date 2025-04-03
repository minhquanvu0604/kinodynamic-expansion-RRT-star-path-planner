#include "dataprocessing.h"
#include <iostream> // Only here for showing the code is working
#include <thread>

DataProcessing::DataProcessing()
{
  radars_.clear(); // ???
}

void DataProcessing::setRadars(std::vector<Radar*> radars){
  radars_=radars;
}

void DataProcessing::findClosestReading(){

  while(true){
    double distance =-1;//In case we have no radar's and someone calls this function
    if(radars_.size()==0){
      // Only here for showing the code is working
      std::cout << "No Radars set:" << __func__ << std::endl;
      std::this_thread::sleep_for (std::chrono::milliseconds(1000));
    }
    else{
      //! We need the maxDistance
      // INSTEAD OF USING A INFINITELY LARGE NUMBER FOR THE INITIAL DISTANCE
      for (unsigned int i=0 ; i< radars_.size();i++){
        double maxVal = radars_.at(i)->getMaxDistance();
        if(maxVal>distance){
          distance=maxVal;
        }
      }

      //! We get the data and run the check here (not keeping data, otherwise
      //! could have stored in member variable data_
      for (unsigned int i=0 ; i< radars_.size();i++){
        std::vector <double> data = radars_.at(i)->getData();
        for(auto elem : data){
          if(elem<distance){
            distance=elem;
          }
        }
      }

      // Only here for showing the code is working
      std::cout << "Closest reading:" << distance << std::endl;

      //! NOTE: However, we have no way of guranteeing this runs as specific rate,
      //! as we are waiting for all radars to provide data, via the getData function
      //! in a loop.
      //!
      //! To address the challenges we had (running at 50ms or every new radar reading)
      //! what shoudl we do?
      //! radar->getData() is blocking and only returns if there is new data, so
      //! do we need to run more threads in here?
    }
  }
}
