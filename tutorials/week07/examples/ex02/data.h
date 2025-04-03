#ifndef DATA_H
#define DATA_H

#include <vector>
#include <mutex>
#include <condition_variable>

namespace pfms {

    struct Data{
        std::vector<int> data; //!< vector of data
        std::mutex mtx; //!< mutex 
        std::condition_variable cv;
    };

}

#endif // DATA_H
