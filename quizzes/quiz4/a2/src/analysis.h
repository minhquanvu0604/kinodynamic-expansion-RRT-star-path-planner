#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <vector>
#include <string>

namespace analysis
{

/**
 * @brief Counts the number of characters in the string supplied
 * @note You have to ignore the terminating character (end character) present in string
 *
 * @param sentece - the string we are analysing
 * @return number of characters
 */
  unsigned int countCharacters(std::string sentence);

  /**
   * @brief Detects a integer number which is part of the string supplied
   * @note Will detect the first number in the string
   *
   * @param sentece - the string we are analysing
   * @return firts number in string
   */
  int getNumber(std::string sentence);
};

#endif // ANALYSIS_H
