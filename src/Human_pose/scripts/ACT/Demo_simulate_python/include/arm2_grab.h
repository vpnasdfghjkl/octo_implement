#ifndef ARM2_FUNCTIONS_H
#define ARM2_FUNCTIONS_H

#include <atomic>
#include <string>
#include <vector>

void arm2_grab(std::atomic<bool>& stopFlag,std::string action_value,std::vector<float> grab_point);
void arm2_loosen(std::atomic<bool>& stopFlag,std::string action_value,std::vector<float> grab_point);

#endif // ARM_FUNCTIONS_H
