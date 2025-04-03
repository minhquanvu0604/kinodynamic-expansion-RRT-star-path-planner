#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <random>    // random number generation
#include <algorithm> // algorithms for sorting
#include <condition_variable>

using namespace std;

//recursive function for fibonacci
int fibonacci(int n)
{
    //if n is zero or one return the number
    if(n<=1)
    {
        return n;
    }
    //recursive call to n-1 and n-2 
    return fibonacci(n-1)+fibonacci(n-2);
}

// The function generates samples
void generateSamples(vector<int> &data, mutex &numMutex, condition_variable& cv) {

  //Setup and seed our random normal distribution generator
  std::default_random_engine generator(std::chrono::duration_cast
                                       <std::chrono::nanoseconds>
                                       (std::chrono::system_clock::now().time_since_epoch()).count());
  std::normal_distribution<> distribution{20, 10}; //generates integeres, mean of 20 and a stdev of 10

  while (true) {

        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        {
            std::unique_lock<mutex> lk(numMutex);
            cout << "sample" << endl;
            // We only access num while the mutex is locked
            int sample = distribution(generator);
            data.push_back(sample);
        }
        //We can unlock here to free up access to mutex
        //lk.unlock();

        cv.notify_one();

        // This delay is included to improve the emulate some other process of generating the data
        // by the sensor which could be at a specific rate (maybe 10Hz - 100ms)
        std::this_thread::sleep_for (std::chrono::milliseconds(100));

    }
}

// This function consumes the samples
void processSamples(vector<int> &data, mutex &numMutex, condition_variable& cv) {
    while (true) {

        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        std::unique_lock<mutex> lk(numMutex);
        cv.wait(lk,[&data]{return !data.empty();});

        // if (!data.empty()){
            int sample = data.back();
            data.pop_back();
            lk.unlock(); // We release mutex here as we don't 
            cout << "fibonacci:" << fibonacci(sample) << endl;
        //   }
        //   else{
        //     numMutex.unlock();
        //   }
    }
}

int main ()
{
    vector<int> data;
    // We will use this mutex to synchonise access to num
    mutex numMutex;

    condition_variable convar;

    // Create the threads
    thread inc_thread(generateSamples,ref(data),ref(numMutex),ref(convar));
    thread print_thread(processSamples,ref(data),ref(numMutex),ref(convar));

    // Wait for the threads to finish (they wont)
    inc_thread.join();
    print_thread.join();

    return 0;
}



