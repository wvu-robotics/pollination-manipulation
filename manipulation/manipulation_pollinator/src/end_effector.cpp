#include <manipulation_pollinator/end_effector.hpp>
#include <unistd.h> // getcwd, getuid
#include <pwd.h> //getpwuid
#include <sys/types.h> //uid_t
#include <std_msgs/Bool.h>

ros::Publisher pub_save_poses;
std::vector<std::vector<float>> createAllCombinations(const std::vector<float>& a, const std::vector<float>& b, const std::vector<float>& c)
{
  int* indices = new int[3];
  std::vector<std::vector<float>> inputVectors;
  std::vector<std::vector<float>> allCombos;
  std::vector<float> temp(3,0);
  inputVectors.push_back(a);
  inputVectors.push_back(b);
  inputVectors.push_back(c);
  int n = inputVectors.size();
  for(int i = 0; i < n; ++i)
  {
    indices[i] = 0;
  }
  while(1)
  {
    temp.clear();
    for(int i = 0; i < n; ++i)
    {
       temp.push_back(inputVectors[i][indices[i]]);
    }
    allCombos.push_back(temp);

    int next = n - 1;
    while(next >= 0 && (indices[next] + 1 >= inputVectors[next].size()))
    {
      --next;
    }

    if(next < 0)
    {
      return allCombos;
    }

    ++indices[next];

    for(int i = next + 1; i < n; ++i)
    {
      indices[i] = 0;
    }
  }
}

std::vector<std::vector<float>> parseValidCombinations(const std::vector<std::vector<float>>& a, const float& MAX_DIFFERENCE)
{
  std::vector<float> possibleCombo;
  std::vector<std::vector<float>> validA;
  std::vector<float> diffs;
  for(int i = 0; i < a.size(); ++i)
      {
        possibleCombo = a[i];
        diffs.clear();
        diffs.push_back(std::fabs(possibleCombo[2] - possibleCombo[1]));
        diffs.push_back(std::fabs(possibleCombo[3] - possibleCombo[1]));
        diffs.push_back(std::fabs(possibleCombo[3] - possibleCombo[2]));
        float greatestDiff = *std::max_element(diffs.begin(), diffs.end());
        if(greatestDiff < MAX_DIFFERENCE)
        {
          validA.push_back(possibleCombo);
        }
       }
  return validA;
}

void save_to_yaml(const std::string _filename, std::vector<float>& _position, const int _posNum)
{
  const char *home_dir = getpwuid(getuid())->pw_dir;

  std::string filename = ros::package::getPath("manipulation_pollinator") + "/data/" + _filename +  std::to_string(_posNum) + ".yaml";
  // std::string filename = std::string(home_dir) + "/pollination_ws/" + _filename +  std::to_string(_posNum) + ".yaml";

  std::fstream file;
  file.open(filename.c_str(), std::fstream::out);

  file << "type: " << "end_effector_pose" << std::endl;

  file << "Actuator_Positions: " << "["
       << _position[0] << ", "
       << _position[1] <<  ", "
       << _position[2] << "]"
       << std::endl;
  file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "end_effector_node");
	ros::NodeHandle nh;
  pub_save_poses = nh.advertise<std_msgs::Bool>("/bramblebee/arm/save_cam_poses", 1);
  maestro actuators;
  //use "ls /dev/serial/by-id" if change maestro board
  printf("init: %d\n", actuators.init("/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00207061-if00"));
  ros::Rate rate(50);
  std::vector<float> actuatorOnePositions;
  std::vector<float> actuatorTwoPositions;
  std::vector<float> actuatorThreePositions;
  std::vector<std::vector<float>> validCombos;
  std::vector<std::vector<float>> allCombos;
  const float INCREMENT_SIZE = 4.0;
  const float MAX_DIFFERENCE = 20.0;
  const int NUM_POSSIBILITES = int(100/INCREMENT_SIZE) + 1;
  for(int i = 0; i < NUM_POSSIBILITES; ++i)
  {
    actuatorOnePositions.push_back(i*INCREMENT_SIZE);
    actuatorTwoPositions.push_back(i*INCREMENT_SIZE);
    actuatorThreePositions.push_back(i*INCREMENT_SIZE);
  }
  std::cout << "Done create vectors" << std::endl;
  allCombos = createAllCombinations(actuatorOnePositions, actuatorTwoPositions, actuatorThreePositions);
  std::cout << "Done create possiblites: " << allCombos.size()<< std::endl;
  validCombos = parseValidCombinations(allCombos, MAX_DIFFERENCE);
  std::cout << "Number of possiblites: " << validCombos.size() << std::endl;
  int posNum = 0;
  while(ros::ok() && posNum < validCombos.size())
  {
    std::cout << "Executing pose: " << posNum << "   ";
    setActuatorPositions(actuators, validCombos[posNum]);
    std_msgs::Bool save_poses_msg;
    // save_poses_msg.data = true;
    // pub_save_poses.publish(save_poses_msg);
    // save_to_yaml("actuatorPositions", validCombos[posNum], posNum);
    sleep(1);
    ++posNum;
    ros::spinOnce();
    rate.sleep();
  }
  setActuatorPositions(actuators, zeroSignals());
  ros::shutdown();
  return 0;
}
