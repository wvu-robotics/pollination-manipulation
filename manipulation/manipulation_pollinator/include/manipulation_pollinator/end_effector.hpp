#ifndef end_effector_h
#define end_effector_h
#include <unistd.h>
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <manipulation_pollinator/maestro.hpp>
#include <ros/package.h>
unsigned short percentageToPWM(float percentage)
{
    if(percentage > 100.0)
    {
        ROS_WARN("tried to command servo to value greater than 100 percent extension");
        return 8000;
    }
    else if(percentage < 0.0)
    {
        ROS_WARN("tried to command servo to value less than 0 percent extension");
        return 4000;
    }
    else
    {
        return (unsigned short)(40.0*percentage + 4000.0);
    }
}

const std::vector<float> zeroSignals()
{
  return {0, 0, 0};
}

void printPositions(maestro& servos)
{
  printf("Position1: %d, ",servos.getPosition(0));
  printf("Position2: %d, ",servos.getPosition(1));
  printf("Position3: %d\n",servos.getPosition(2));
}

void setActuatorPositions(maestro& servos, const std::vector<float>& v)
{
  servos.setPosition(0,percentageToPWM(v[0]),percentageToPWM(v[1]),percentageToPWM(v[2]));
  printPositions(servos);
}
#endif
