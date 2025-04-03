// Includes std::cout and friends so we can output to console
#include <iostream>
// Includes vector STL
#include <vector>
// Using stringstream to get the size if passed via comand line
#include <sstream>


// Every executable needs a main function which returns an int
int main (int argc,char** argv) {

    //Ex04
    int length=10; //Let's say it is 10 members long by default

    //! Obtain value of vector size if supplied via command line
    //! ie running the exectubale (./vectors 5) will create a vector with 5 elements (otherwise default is 10)
    //! https://stackoverflow.com/questions/2797813/how-to-convert-a-command-line-argument-to-int
    if(argc >1){
      std::stringstream ss(argv[1]);
      if (!(ss >> length)) { // Checks if we can convert the strinstream to a integer
        std::cerr << "Invalid number: " << argv[1] << " length of zero used \n";
      } else if (!ss.eof()) {
        std::cerr << "Trailing characters after number: " << argv[1] << " using :" << length << "\n";
      }
    }

    //! Vector
    //! ------

    //! Create a vector if integers called  `data`
    std::vector<int> data;


    //! Create a loop to populate elements of `data` (each element [i] =i))
    for (int i=0;i<length;i++){
        data.push_back(i);
    }

    //! Loop to print elements of vector (let's acess each element location
    for (int i=0;i<data.size();i++){
        std::cout << data.at(i) << " ";
    }
    std::cout << std::endl;

    //! Loop to print elements of vector (let's use auto)
    for (auto elem : data){
        std::cout << elem << " ";
    }
    std::cout << std::endl;

    //! How to create a vector of floats?
    //  To answer this look how we defined the vector of integers, what do we need to change

    //! BONUS QUESTION
    // How would you create the vector in a function?

    // Let's have a look at a Vector of Vectors (Container of Data)
    //------
    {
      //* Create a vector of vectors containing integers called  `data`
      std::vector<std::vector<int>> data;

      //* Create a loop to populate elements of `data` such that is is a matrix of 4x4 elements (each element row =i))
      const int rows=4; //We have 4 rows
      const int cols=4; //We have 4 cols

      for (int i=0;i<rows;i++){
          std::vector<int> row;
          for (int j=0;j<cols;j++){
              row.push_back(j);
          }
          data.push_back(row);
      }

      std::cout << "MATRIX" << std::endl;

      //* Loop to print elements of `data`

      for (int i=0;i<data.size();i++){
          for (int j=0;j<data.at(i).size();j++){
              std::cout << data.at(i).at(j) << " ";
          }
          std::cout << std::endl;
      }

      //* What else could you store in this container?
      // Contemplate ..
      // Does this need to be a square matrix?
      // Why do we the curly braces above the declaration of std::vector<std::vector<int>> data;?


    }

    return 0;
}




