#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <manipulation_vision/relative_frequencies.hpp>

int main(int argc, char** argv)
{
  // Set up ROS
  ros::init(argc, argv, "generate_lookup_node");
  ros::NodeHandle nh;

  // Print indicator node is running
  ROS_INFO("generate_lookup_node running...");
  ROS_INFO("generating lookup table...");

  /*
    LOAD RELATIVE FREQUENCIES
  */
  std::vector<double> nL = return_negative_likelihood(); // likelihoods given negative label, p(r|l=0),p(g|l=0),p(b|l=0)
  std::vector<double> pL = return_positive_likelihood(); // likelihoods given positive label, p(r|l=1),p(g|l=1),p(b|l=1)
  double nM = return_negative_prior(); // prior for negative label, p(l=0) (marginal from example images)
  double pM = return_positive_prior(); // prior for positive label, p(l=1) (marginal from example images)

  /*
    GENERATE LOOKUP TABLE
  */

  /*
    ALLOCATE MEMORY FOR LOOK-UP TABLE
  */
  //allocate 3D array for look-up table
  const int RDIM = 256;
  const int GDIM = 256;
  const int BDIM = 256;

  unsigned char ***H = new unsigned char**[RDIM];
  for (int i = 0; i < RDIM; ++i)
  {
    H[i] = new unsigned char*[GDIM];
    for (int j = 0; j < GDIM; ++j)
    {
      H[i][j] = new unsigned char[BDIM];
    }
  }

  /*
    GENERATE LOOK-UP TABLE
  */
  //assign class label to each element of look-up table that maximizes the posterior
  double nP = 0;
  double pP = 0;
  for (int i=0; i<RDIM; i++)
  {
    for (int j=0; j<GDIM; j++)
    {
      for (int k=0; k<BDIM; k++)
      {
        nP = nM*nL[i]*nL[j+256]*nL[k+512]; //posterior given negative class label, p(l=0,x)
        pP = pM*pL[i]*pL[j+256]*pL[k+512]; //posterior given positive class label, p(l=1,x)

        if(pP > 0.9*nP)
        {
          H[i][j][k] = 255;
        }
        else
        {
          H[i][j][k] = 0;
        }
      }
    }
  }

  /*
    WRITE LOOK-UP TABLE TO FILE
  */
  //write look-up table to .csv file (overwrites existing file)
  std::string filepath = ros::package::getPath("manipulation_vision") + "/data/lookup_table.csv";
  std::ofstream logger;
  logger.open(filepath.c_str(), std::ofstream::out | std::ofstream::trunc);
  for (int i=0; i<RDIM; i++)
  {
    for (int j=0; j<GDIM; j++)
    {
      for (int k=0; k<BDIM; k++)
      {
        if(i==0 && j==0 && k==0) logger << (int)H[i][j][k];
        else logger << " " << (int)H[i][j][k];
      }
    }
  }
  logger.close();

  /*
    FREE MEMORY/END PROGRAM
  */
  //de-alloc look-up table
  for (int i = 0; i < RDIM; ++i)
  {
    for (int j = 0; j < GDIM; ++j)
    {
      delete [] H[i][j];
    }
    delete [] H[i];
  }
  delete [] H;

  return 0;
}
