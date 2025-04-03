#include <gtest/gtest.h>
#include <climits>

//This tool allows to identify the path of the package on your system
#include <ros/package.h>

//These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

//We include our header file
#include "../src/grid_processing.h"

//==================== HELPER FUNCTIONS ====================//
void printCells(nav_msgs::OccupancyGrid grid) {
  for (unsigned int row=0;row<grid.info.height-1;row++){
    for (unsigned int col=0;col<grid.info.width-1;col++){
      unsigned int index=(row*grid.info.height)+col;
      std::cout << static_cast<int>(grid.data.at(index)) << " ";
    }
    std::cout << std::endl;
  }

}
//==================== HELPER FUNCTIONS ====================//



TEST(GeneratedOgMap,ConnectionTest){

    //! The below code tests line connectivity
    //! On a occupied, free, and partial free OgMap

    float mapSize=4.0;
    float resolution=0.1;

    unsigned int size_x = static_cast<unsigned int>(mapSize/resolution);
    unsigned int size_y = static_cast<unsigned int>(mapSize/resolution);

    //Here we generate a new OgMap message
    nav_msgs::OccupancyGrid grid;
    grid.info.width = size_x;
    grid.info.height = size_y;
    grid.info.resolution = resolution;
    grid.data.assign(size_x * size_y, -1); //And we create a vector os size_x * size_y values, with each element of vector assigned value -1

    // Let's check the first cell in OgMap is -1
    ASSERT_EQ(-1,grid.data.at(0));

    // ^ y
    // |
    // |
    // o------> x
    //

    // We have to initialse each part of the structire separately : https://github.com/ros/ros_comm/issues/148
    geometry_msgs::Point origin;//Origin is a struct
    origin.x=0.1;origin.y=0.6;origin.z=0;
    geometry_msgs::Point dest;
    dest.x=1.5;dest.y=1.5;dest.z=0;

    GridProcessing gridProcessing(grid);

    // Should the OgMap allow the origin and destination to be connected in a straight line if all point are occupied
    ASSERT_FALSE(gridProcessing.checkConnectivity(origin,dest));

    //Let's augument the OgMap now to be unoccupied
    grid.data.assign(size_x * size_y, 0); //each element of vector assigned value 0 (free space)

    // Let's check the first cell in OgMap is 0
    ASSERT_EQ(0,grid.data.at(0));

    gridProcessing.setGrid(grid);//Let's set this new grid now in our gridProcessing object

    // Does the image now allow the origin and destination to be connected with a straight line if all cells are free?
    ASSERT_TRUE(gridProcessing.checkConnectivity(origin,dest));


    // Let's now be a bit more systematic, how about an OgMap that is all free cells
    // Apart from a line that goes down the middle (the y axis)?
    // Let's make a line of occupied cells down the middle
    std::vector<int8_t>& map_data (grid.data); // In this step we now have the actual cell data inside map_data

    for (unsigned int row=0;row<grid.info.height;row++){//This now gives us now the row
      //It's a row major
      //Our ine will go through the middle, so the column in middle
      unsigned int index = row*grid.info.width + (grid.info.width/2);
      map_data.at(index)=100;
    }
    gridProcessing.setGrid(grid);//Let's set this new grid now in our gridProcessing object

    //printCells(grid);

    // Let's pick a point on either size of the y axis
    origin.x=-1.5;origin.y=0.0;
    dest.x=1.5;dest.y=0.0;

    // Does the image now allow the origin and destination to be connected with a straight line, it shoudl NOT!
    ASSERT_FALSE(gridProcessing.checkConnectivity(origin,dest));

    /**
     * @todo - Ex01 : Create an OgMap used for connectivity testing as per specification
     *
     * - Create OgMap as per supplied drawing
     * - Select origin/destination points that will sucseed and fail in the test
     *
     */

    //We reset it to all zeros
    grid.data.assign(size_x * size_y, 0);
    {
      std::vector<int8_t>& map_data (grid.data); // In this step we now have the actual cell data inside map_data

      for (unsigned int row=0;row<grid.info.height;row++){//This now gives us now the row
        //It's a row major

	//! You now need to use the fact you have a loop, to modify the map 
	//! to look like what was supplied in the image

      }
    }
    gridProcessing.setGrid(grid);//Let's set this new grid now in our gridProcessing object

    //printCells(grid);

    //let's pick a point on either size of the y axis
    origin.x=-1.5;origin.y=0.0;
    dest.x=1.5;dest.y=0.0;

    // Does the ogmap now allow the origin and destination to be connected with a straight line, it shoudl!
    ASSERT_TRUE(gridProcessing.checkConnectivity(origin,dest));

    //let's pick a point on either size of the y axis
    origin.x=-1.5;origin.y=-1.5;
    dest.x=1.5;dest.y=1.5;

    // Does the image now allow the origin and destination to be connected with a straight line, it shoudl NOT!
    ASSERT_FALSE(gridProcessing.checkConnectivity(origin,dest));

}

TEST(LoadedOgMap,ConnectionTest){

  //! The below code tests the OgMap analysis on a sample OgMap
  //! The data has been saved in a bag, that is opened and used.


  //! Below command allows to find the folder belonging to a package (week11 is the package name not the folder name)
  std::string path = ros::package::getPath("week11");
  //! Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/samples/";
  //! The file is called smiley.bag
  std::string file = path + "sample.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid

  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    if(m.getTopic() == "/local_map/local_map"){
      grid = m.instantiate<nav_msgs::OccupancyGrid>();
    }
    if (grid != nullptr){
      // Now we have the first grid in bag so we can proceed
      break;
    }
  }

  bag.close();

  ASSERT_NE(grid, nullptr);//Check that we have a grid from the bag

  GridProcessing gridProcessing(*grid);

  geometry_msgs::Point origin;//Origin is a struct
  origin.x=0.0;origin.y=0.0;
  geometry_msgs::Point dest;
  dest.x=-4.0;dest.y=0.0;

  // Should the OgMap allow the origin and destination to be connected in a straight line if all point are occupied
  ASSERT_FALSE(gridProcessing.checkConnectivity(origin,dest));

  origin.x=0.0;origin.y=0.0;
  dest.x=0.0;dest.y=3.0;

  // Should the OgMap allow the origin and destination to be connected in a straight line if all point are occupied
  ASSERT_TRUE(gridProcessing.checkConnectivity(origin,dest));

  /**
   * @todo - Ex02 : Utilise OgMap stored in bag for testing
   *
   * - Select origin/destination points that will succeed and fail in the test
   *
   */

}

/**
 * @todo - Ex03 : Save a new rosbag with an OgMap
 *
 * - Use the additional OgMap for testing
 * - Create a unit test
 * - Select origin/destination points that will succeed and fail in the test
 *
 */


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
