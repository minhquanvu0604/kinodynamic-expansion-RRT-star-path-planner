#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

// header files needed from our libraries, because of include_directories in CMakeLists.txt we don't need the ..
// before these filenames
#include "tf2.h"
#include "tf.h"
#include "types.h"
#include "analysis.h"

using namespace std;
using geometry_msgs::Pose;
using geometry_msgs::RangeBearingStamped;
using std::vector;

TEST (Transforms, Local2Global) {


    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(0.785398);

    {
        Point bogie = {3171.34,-4574.67,0};
        RangeBearingStamped rb = {5566.41,4.53316,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);
    }

    {
        Point bogie = {-3288.52,-2516.65,0};
        RangeBearingStamped rb = {4141,3.0094,0};

        Point bogieComputed = tf2::local2Global(rb,aircraft);

        EXPECT_NEAR(bogie.x,bogieComputed.x,0.5);
        EXPECT_NEAR(bogie.y,bogieComputed.y,0.5);

    }

}


TEST (Analysis, Time2Impact ) {

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(M_PI/4);

    vector<Point> goals;
    goals.push_back({77.59,14.77,0});
    goals.push_back({17.77,-15.37,0});
    goals.push_back({-30,46.67,0});
    goals.push_back({-20.27,-33.39,0});
    goals.push_back({-41.31,-18.05,0});

    Analysis analysis(goals);
    vector<double> impactTimes = analysis.timeToImpact(aircraft);

    vector<double> impactTimesCorrect ={79.5806,24.9934,56.8372,41.9628,47.8494};

    //Unfortunately have to test each element with a tolerance
    ASSERT_EQ(impactTimes.size(),impactTimesCorrect.size());
    for (unsigned int i=0;i<impactTimes.size();i++){
        EXPECT_NEAR(impactTimes.at(i),impactTimesCorrect.at(i),1e-3);
    }

}

// TEST (Analysis, ExportGraph){

//     vector<Point> goals;
//     goals.push_back({77.59,14.77,0});   // 0
//     goals.push_back({17.77,-15.37,0});  // 1
//     goals.push_back({-30,46.67,0});     // 2
//     goals.push_back({-20.27,-33.39,0}); // 3
//     goals.push_back({-41.31,-18.05,0}); // 4

//     double lengths[5][5];
//     lengths[0][1] = 66.98;
//     lengths[0][2] = 112.22;
//     lengths[0][3] = 109.07;
//     lengths[0][4] = 123.35;
//     lengths[1][2] = 78.3;
//     lengths[1][3] = 42.1;
//     lengths[1][4] = 59.14;
//     lengths[2][3] = 80.65;
//     lengths[2][4] = 65.7;
//     lengths[3][4] = 26.04;



//     Analysis analysis(goals);
//     AdjacencyList graph = analysis.exportGraph();

//     ASSERT_EQ(graph.size(), goals.size());

//     for (int point = 0; point < graph.size(); point++){
//         // cout << "Point ID: " << point << endl;
//         for (int edge = 0; edge < graph.size(); edge++){
//                 // cout << "Edge from " << point << "to " << edge << " : Length:" <<
//                     // graph.at(point).at(edge).first << " and Connect to:" << 
//                     // graph.at(point).at(edge).second << endl;
//             if (point > edge) EXPECT_NEAR(graph.at(point).at(edge).first,lengths[edge][point],1e-2);
//             else if (point < edge) EXPECT_NEAR(graph.at(point).at(edge).first,lengths[point][edge],1e-2);
        
//         }
//     }
// }


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
