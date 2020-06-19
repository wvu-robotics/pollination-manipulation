#include <manipulation_pollinator/pollinator_control.hpp>

namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
PollinatorControl::PollinatorControl()
{
    //start service
    pollinateFlowersServ = nh.advertiseService("pollinate_flower", &PollinatorControl::pollinateFlower, this);

    //initializes maestro board
    printf("init: %d\n", actuators_.init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00207061-if00"));
    setActuatorPositions(actuators_, zeroSignals());
}

//----------------------------------------------------------------------------
/**
 * Load lookup table for inverse kinematics
 */

bool PollinatorControl::
loadLookupTableMaestro ()
{
  std::string filename = ros::package::getPath("manipulation_pollinator") + "/data/ik_lut.csv";

  std::ifstream in(filename.c_str());
  if (!in.is_open()) std::cout<< "File did not open" << std::endl;

  std::vector<std::string> values;
  std::string line;

  while (std::getline(in,line))
  {
      Tokenizer tok(line);
      values.assign(tok.begin(), tok.end());
      key_.push_back({std::stof(values[3]), std::stof(values[4]), std::stof(values[4]),
                      std::stof(values[5]), std::stof(values[6]), std::stof(values[7]),
                      std::stof(values[8]), std::stof(values[9])});
      lut_[key_.back()] = {std::stof(values[0]), std::stof(values[1]), std::stof(values[2])};
  }

  return true;
}

//----------------------------------------------------------------------------
/**
 *
 */
bool PollinatorControl::
pollinateFlower (manipulation_pollinator::PollinateFlower::Request  &req,
                 manipulation_pollinator::PollinateFlower::Response &res)
{
  //for now hard code setpoint
  req.setpoint = {0.004906,
                 -0.003918,
                 -0.020224,
                  0.997504,
                 -0.009017,
                 -0.014515,
                 -0.068504};

  //do orbital motion
  orbitPose(lut_[getClosestPose(req.setpoint)]);

  return true;
}


//----------------------------------------------------------------------------
/**
 *
 */

std::vector<float> PollinatorControl::
getClosestPose(std::vector<float> & pose)
{
  float MIN_ERROR = std::numeric_limits<float>::max();
  int location;
  for(int i = 0; i < key_.size(); ++i)
  {
    float totalError = 0;
    for(int j = 0; j < 3; ++j)
    {
      totalError += std::pow((key_[i][j] - pose[j]),2);
    }
    if(totalError < MIN_ERROR)
    {
      MIN_ERROR = totalError;
      location = i;
    }
  }
  return key_[location];
}

//----------------------------------------------------------------------------
/**
 *
 */
void PollinatorControl::
orbitPose(std::vector<float>& pose)
{
  float LENGTH_ORBIT = 20.0;
  int NUM_ORBITS = 4;
  std::vector<float> newPose = pose;
  for(int i = 0; i < NUM_ORBITS; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      newPose = pose;
      if(newPose[j] + LENGTH_ORBIT > 100)
      {
        newPose[j] -= LENGTH_ORBIT;
      }
      else
      {
        newPose[j] += LENGTH_ORBIT;
      }
      setActuatorPositions(actuators_,newPose);
      sleep(1);
    }
  }
      setActuatorPositions(actuators_, zeroSignals());
}

}
