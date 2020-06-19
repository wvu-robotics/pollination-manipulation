#include <ros/ros.h>
#include <ros/package.h>

#include <manipulation_pollinator/PollinateFlower.h>
#include <manipulation_pollinator/end_effector.hpp>

#include <boost/tokenizer.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <string>
#include <algorithm>
#include <iterator>


namespace manipulation
{

template <typename Container>
struct container_hash {
    std::size_t operator()(Container const& c) const {
        return boost::hash_range(c.begin(), c.end());
    }
};

typedef boost::unordered_map<std::vector<float>, std::vector<float>, container_hash<std::vector<float>>> map;
typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;

class PollinatorControl {
  public:
    PollinatorControl();

    bool loadLookupTableMaestro ();
    bool pollinateFlower       (manipulation_pollinator::PollinateFlower::Request  &req,
                                manipulation_pollinator::PollinateFlower::Response &res);

    ros::NodeHandle    nh;
    ros::ServiceServer pollinateFlowersServ;

  private:

    std::vector<float> getClosestPose(std::vector<float>& pose);
    void orbitPose(std::vector<float>& pose);

    std::vector<std::vector<float>> key_;
    map lut_;
    maestro actuators_;
};

}
