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


TEST (Transform, Global2Local ) {

    Pose aircraft;
    aircraft.position = {-2000,1100,0};
    aircraft.orientation = tf::yawToQuaternion(1.0472);

    {
        Point bogie = {-3614.49,2759.74,0};
        RangeBearingStamped rb;
        rb = {2315.45,1.29518,0};

        RangeBearingStamped rbComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rbComputed.range,rb.range,0.5);
        EXPECT_NEAR(rbComputed.bearing,rb.bearing,1e-3);
    }

    {
        Point bogie = {-4842.62,-1380.23,0};
        RangeBearingStamped rb = {3772.53,2.81182,0};

        RangeBearingStamped rbComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rbComputed.range,rb.range,0.5);
        EXPECT_NEAR(rbComputed.bearing,rb.bearing,1e-3);

    }


    {

        aircraft.position = {-2000,1100,0};
        aircraft.orientation = tf::yawToQuaternion(-1.0472);

        Point bogie = {-1930.32,2438.4,0};
        RangeBearingStamped rb = {1340.22,2.56598,0};

        RangeBearingStamped rbComputed = tf2::global2local(bogie,aircraft);

        EXPECT_NEAR(rbComputed.range,rb.range,0.5);
        EXPECT_NEAR(rbComputed.bearing,rb.bearing,1e-3);

    }

}


TEST (Analysis, ExportGraph ) {

    Pose aircraft;
    aircraft.position = {0,0,0};
    aircraft.orientation = tf::yawToQuaternion(M_PI/4);

    vector<Point> goals;
    goals.push_back({771.59,1451.77,0});
    goals.push_back({1715.77,-1527.37,0});
    goals.push_back({-3026,464.671,0});
    goals.push_back({-2066.27,-3301.39,0});
    goals.push_back({-4100.31,-1899.05,0});

    Analysis analysis(goals);
    AdjacencyList graph = analysis.exportGraph();

    ASSERT_EQ(graph.size(),goals.size());
    ASSERT_EQ(graph.at(0).size(),goals.size()-1);
    ASSERT_EQ(graph.at(1).size(),goals.size()-1);
    ASSERT_EQ(graph.at(2).size(),goals.size()-1);
    ASSERT_EQ(graph.at(3).size(),goals.size()-1);
    ASSERT_EQ(graph.at(4).size(),goals.size()-1);

    //Random checks here 0 connects to 1 and 1 to zero
    ASSERT_EQ(graph.at(0).at(0).second,1);
    ASSERT_EQ(graph.at(1).at(0).second,0);
    ASSERT_NEAR(graph.at(1).at(0).first,3125.18,1.0);
    ASSERT_NEAR(graph.at(0).at(0).first,3125.18,1.0);

    //Random checks here 0 connects to 4 and 4 to zero
    ASSERT_EQ(graph.front().back().second,4);
    ASSERT_EQ(graph.back().front().second,0);
    ASSERT_NEAR(graph.front().back().first,5912.99,1.0);
    ASSERT_NEAR(graph.back().front().first,5912.99,1.0);

//    int node =0;
//    for (auto edges : graph){
//        std::cout << node++ << std::endl;
//        for(auto edge : edges){
//            std::cout << edge.second << " " << edge.first << std::endl;
//        }
//    }

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
