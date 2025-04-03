#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>

using namespace std;

void incrementNum(int &num, mutex &numMutex) {
    while (true) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        //numMutex.lock();
      {
        std::unique_lock<std::mutex> lck(numMutex);
        //cout << "num++" << endl;
        // We only access num while the mutex is locked
        num++;
      }
        //numMutex.unlock();
        // This delay is included to improve the readability of the program output
        // and avoid hard-looping and consuming too much cpu
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }
}

void printNum(int &num, mutex &numMutex) {
    while (true) {
        // We can only obtain a lock in this thread if the mutex
        // is not locked anywhere else
        //numMutex.lock();
      {
        std::unique_lock<std::mutex> lck(numMutex);
        // We only access num while the mutex is locked
        cout <<  "num is:" << num << endl;
      }
        //numMutex.unlock();
        // This delay is included to improve the readability of the program output
        // and avoid hard-looping and consuming too much cpu
        // A better solution would be to use a condition_variable to make
        // this thread wait for a notification from the incrementing thread
        std::this_thread::sleep_for (std::chrono::milliseconds(50));
    }
}

int main ()
{
    int num = 0;
    // We will use this mutex to synchonise access to num
    mutex numMutex;

    // Create the threads
    thread inc_thread(incrementNum,ref(num),ref(numMutex));
    thread print_thread(printNum,ref(num),ref(numMutex));

    // Wait for the threads to finish (they will not in this implementation)
    inc_thread.join();
    print_thread.join();

    return 0;
}



