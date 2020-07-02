#include <manipulation_vision/classification.hpp>

namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
Classification::Classification()
{
    classificationServ = nh.advertiseService("classify_image", &Classification::classifyImage, this);
    poseServ = nh.advertiseService("classify_pose", &Classification::classifyPose, this);
}

//----------------------------------------------------------------------------
/**
 * Server for performing classification on images
 */

bool Classification::classifyImage(manipulation_vision::ClassifyFlowers::Request  &req,
                                   manipulation_vision::ClassifyFlowers::Response &res)
{
  // Paths for image output and executable location
  std::string outputPath;
  std::string execPath;

  // Set command to add executable location to python path
  execPath = ros::package::getPath("manipulation_vision") + "/src/";
  std::string execCommand = "sys.path.append('" + execPath + "')";
  const char* pyExecCommand = execCommand.c_str();
  
  for(int i=0; i<req.numberOfSegments; i++)
  {
    outputPath += ros::package::getPath("manipulation_vision") + "/output/contour" + std::to_string(i) + ".png;";
  }

  const char* picpath = outputPath.c_str();
  Py_Initialize();
  if ( !Py_IsInitialized() ) {
      return -1;
  }
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(pyExecCommand);
  PyObject* pMod = NULL;
  PyObject* pFunc = NULL;
  PyObject* pParm = NULL;
  PyObject* pRetVal = NULL;
  char* iRetVal;
  const char* modulName="classify";    //this is the name of the called py file module
  pMod = PyImport_ImportModule(modulName);
  if(!pMod)
  {
      std::cout<<"Import module failed!";
      PyErr_Print();
      return -1;
  }
  const char* funcName="evaluate_multipleString";  //this is the name of the function called in this py file module
  //const char* funcName="evaluate_singleString";
  pFunc = PyObject_GetAttrString(pMod, funcName);
  if(!pFunc)
  {
      std::cout<<"Import function failed!";
      return -2;
  }
  pParm = PyTuple_New(1);
  PyTuple_SetItem(pParm, 0, Py_BuildValue("s",picpath));//the incoming parameter is the path of the picture
  pRetVal = PyEval_CallObject(pFunc, pParm);//execute the py script here
  PyArg_Parse(pRetVal, "s", &iRetVal);//py script returns value to iRetVal
  std::cout<< "image classification result:\n"; // 0 refers to flower, 1 refers to non-flower
  std::cout<< iRetVal << "\n";

  Py_DECREF(pMod);
  Py_DECREF(pFunc);
  Py_DECREF(pParm);
  Py_DECREF(pRetVal);
  // close Python
  // Py_Finalize();

  //Return result
  std::string s = iRetVal;
  std::string delim = ";";
  res.responseProbabilities.clear();
  auto start = 0U;
  auto end = s.find(delim);
  while (end != std::string::npos)
  {
      //std::cout << s.substr(start, end - start) << std::endl;
      res.responseProbabilities.push_back(std::stod(s.substr(start, end - start)));
      start = end + delim.length();
      end = s.find(delim, start);
  }
  //std::cout << s.substr(start, end);
	std::cout<< "classification ended \n";
  res.success = true;

  return true;
}

//----------------------------------------------------------------------------
/**
 * Server for performing classification on images
 */
bool Classification::classifyPose(manipulation_vision::ClassifyPose::Request  &req,
                                  manipulation_vision::ClassifyPose::Response &res)
{

  std::string filepath = ros::package::getPath("manipulation_vision") + "/output/contour" + std::to_string(req.flowerIndex) + ".png;";


  // Set command to add executable location to python path
  std::string execPath = ros::package::getPath("manipulation_vision") + "/src/";
  std::string execCommand = "sys.path.append('" + execPath + "')";
  const char* pyExecCommand = execCommand.c_str();

  const char* picpathPose = filepath.c_str();
  Py_Initialize();
  if ( !Py_IsInitialized() ) {
      return -1;
  }
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(pyExecCommand);
  PyObject* pModPose = NULL;
  PyObject* pFuncPose = NULL;
  PyObject* pParmPose = NULL;
  PyObject* pRetValPose = NULL;
  char* iRetValPose;
  const char* modulNamePose="classify";    //这个是被调用的py文件模块名字
  pModPose = PyImport_ImportModule(modulNamePose);
  if(!pModPose)
  {
      std::cout<<"Import module failed!";
      PyErr_Print();
      return -1;
  }
  const char* funcNamePose="evaluate_pose";  //这是此py文件模块中被调用的函数名字
  pFuncPose = PyObject_GetAttrString(pModPose, funcNamePose);
  if(!pFuncPose)
  {
      std::cout<<"Import function failed!";
      return -2;
  }
  pParmPose = PyTuple_New(1);
  PyTuple_SetItem(pParmPose, 0, Py_BuildValue("s",picpathPose));//传入的参数，是图片的路径
  pRetValPose = PyEval_CallObject(pFuncPose, pParmPose);//这里开始执行py脚本
  PyArg_Parse(pRetValPose, "s", &iRetValPose);//py脚本返回值给iRetVal
  std::cout<< "pose classification result:\n"; // 0,1,2 refers to c3, c2, c1 respectively.
  std::cout<< iRetValPose << "\n";

  Py_DECREF(pModPose);
  Py_DECREF(pFuncPose);
  Py_DECREF(pParmPose);
  Py_DECREF(pRetValPose);
  // close Python
  // Py_Finalize();

  //Return result
  std::string s = iRetValPose;
  std::string delim = ";";
  res.responseProbabilities.clear();
  auto start = 0U;
  auto end = s.find(delim);
  while (end != std::string::npos)
  {
      //std::cout << s.substr(start, end - start) << std::endl;
      res.responseProbabilities.push_back(std::stod(s.substr(start, end - start)));
      start = end + delim.length();
      end = s.find(delim, start);
  }
  //std::cout << s.substr(start, end);
std::cout<< "classification pose ended \n";
  res.success = true;

  return true;
}

}
