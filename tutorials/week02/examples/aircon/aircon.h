#ifndef AIRCON_H
#define AIRCON_H

#include <chrono>
#include <string>

using namespace std::chrono;//Let's us refer to system_clock without stating std::chrono

// Aircon unit has a few states
namespace aircon{
    enum State
    {
        IDLE,       //Neither heating nor cooling
        COOLING,    // Cooling
        HEATING     // Heating
    };
}

class Aircon
{
public:
    /**
     * @brief Creates aircon, defaults to running it for 3minutes, 
     * Requires executing run function to peform task, becuase we do not know threading yet :)
     */
    Aircon();

    /**
     * @brief Indicates if desired temperature reached
     * @return desired temperature reached
     */
    bool tempReached();

    /**
     * @brief Sets desired temperature
     * @param temp - desired temperature [C]
     */
    bool setDesiredTemp(double temp);

    /**
     * @brief Returns current temeprature
     * @return current temperature [C]
     */

    double getCurrentTemp(); //! Returns current temperature

    /**
     * @brief Set running time (total runing time)
     * Unit will run until it is IDLE (finhished heating/cooling) AND this time is reached
     * @param timeS - Running time is [s]
     */
    void setRunTime(int timeS);

    bool run(void); //! Run the air-condition, blocking call which takes 5s (one cycle)

    /**
     * @brief Get Information on aircon as a string
     * @return string containing time from starting, state, current temperature
     */
    std::string getInfoString(void);
private:
    /**
     * @brief Time in the unit has been running
     * @return time unit has been running (since starting) in [s]
     */
    double elapsedTime();
    void heating(); //! Heat (increases temperature)
    void cooling(); //! Cool (decreases temperature)
    /**
     * @brief Delays execution by supplied time
     * @param ms - time to delay execution in [ms]
     */
    void sleepMs(int ms);

    aircon::State state_; //! Current state
    bool heating_active_; //! Indicates if heating is active (theer is a delay to start heating)
    double desired_temp_; //! Desired temperature
    double current_temp_; //! Current temperature
    system_clock::time_point start_time_; //! Start time the air-con was activated
    double run_time_; //! Desired running time [s]

};

#endif // AIRCON_H
