#include "display_race.h"
#include <cmath>
//
//Student do not need to alter this file, purely for visualisation
//

//Colours of actors 
static const cv::Scalar CLR_MAP       = cv::Scalar(255, 160, 160);
static const cv::Scalar CLR_BSTATION  = cv::Scalar(0, 0, 0);
static const cv::Scalar CLR_CAR  = cv::Scalar(0, 0, 255);
static const cv::Scalar CLR_TEXT  = cv::Scalar(0, 200, 255);

DisplayRace::DisplayRace(std::vector<CarInterface*> cars):
    cars_(cars)
{
    track_ = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC3, CLR_MAP);

    cv::namedWindow("track",cv::WINDOW_NORMAL);
    cv::imshow("track", track_);
    cv::waitKey(1); //WaitKey is unavoidable. It will insert a 1ms delay.

}

void DisplayRace::updateDisplay(){

    //Draw the empty race track
    track_ = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC3, CLR_MAP);

    //Draw the centre
    int radius      =1;
    int thickness   =1;
    cv::circle(track_, cv::Point(MAP_CENTRE,MAP_CENTRE),radius,CLR_BSTATION,thickness);

    //Draw the race marking
    cv::circle(track_, cv::Point(MAP_CENTRE,MAP_CENTRE),RADIUS+5,CLR_BSTATION,3);
    cv::circle(track_, cv::Point(MAP_CENTRE,MAP_CENTRE),RADIUS-5,CLR_BSTATION,3);

    std::string info = "centre";
    cv::putText(track_, info, cv::Point(MAP_CENTRE,MAP_CENTRE),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, CLR_TEXT);

    for (unsigned int i=0;i<cars_.size();i++){
      double odo =cars_.at(i)->getOdometry();
      int loops = (int)(odo/(2*M_PI*RADIUS));
      double remain = odo-(loops*(2*M_PI*RADIUS));
      double angle =  remain/(double)(RADIUS);
      int x=static_cast<int>(RADIUS*cos(angle));
      int y=static_cast<int>(RADIUS*sin(angle));

      cv::circle(track_, cv::Point(MAP_CENTRE+x,MAP_CENTRE+y),1,CLR_CAR,3);
      cv::putText(track_, cars_.at(i)->getMake(), cv::Point(MAP_CENTRE+x,MAP_CENTRE+y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, CLR_TEXT);

    }

    cv::imshow("track", track_);
    cv::waitKey(1); //WaitKey is unavoidable. It will insert a 1ms delay.

}
