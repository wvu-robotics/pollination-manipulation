#include <manipulation_mapping/flower_mapper.hpp>
#include <parameter_utils/ParameterUtils.h>

namespace pu = parameter_utils;
using gtsam::Rot3;
using gtsam::Pose3;
using gtsam::Point3;
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::Vector4;
using gtsam::Vector6;
using gtsam::PriorFactor;
using gtsam::BetweenFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::LevenbergMarquardtOptimizer;


namespace manipulation
{

//----------------------------------------------------------------------------
/**
 * Constructor
 */
FlowerMapper::
FlowerMapper()
{
        load_parameters(nh);
        //server
        clearFlowerMapServ = nh.advertiseService("clear_flower_mapper", &FlowerMapper::clear_flower_map, this);
        constructFlowerMapServ = nh.advertiseService("update_flower_mapper", &FlowerMapper::update_flower_map, this);
}

//----------------------------------------------------------------------------
/**
 * Server for updating flower map given flower observations
 */
bool FlowerMapper::
clear_flower_map (manipulation_common::UpdateFlowerMap::Request  &req,
                  manipulation_common::UpdateFlowerMap::Response &res)
{
        res.status = 0;
        if (req.clear_map)
        {
          map_.clear();
          graphs_.clear();
        }
        res.status = 1;
        return true;
}

/**
 * Server for updating flower map given flower observations
 */
bool FlowerMapper::
update_flower_map (manipulation_common::UpdateFlowerMap::Request  &req,
                   manipulation_common::UpdateFlowerMap::Response &res)
{
        res.status = 0;
        std::vector<double> flower_probs; //this will be loaded from req
        std::cout << "req.poses.size() = " << req.poses.size() << std::endl;
        for(int i=0; i<req.poses.size(); i++)
        {
                gtsam::Pose3 pose_est = pose_msg_to_gtsam(req.poses[i]);
                pc_ = req.point_clouds[i];
                normal_ = get_pc_norm_vec(pc_);
                int flower_index = get_flower_index(pose_est);
                update_graph(flower_index, pose_est.translation(), normal_, 0.5);
        }

        map_pub_ = nh.advertise<manipulation_common::FlowerMap>( "flower_map", 10, false);
        vec_pub_ = nh.advertise<visualization_msgs::MarkerArray>( "flower_vec", 10, false);
        pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("flower_pts", 10, false);

        send_flower_map(res, map_pub_);
        send_flower_vec(res, vec_pub_);

        res.status = 1;
        return true;
}

//////////////////////////
// Start Helper Methods //
//////////////////////////
gtsam::Pose3 FlowerMapper::pose_msg_to_gtsam(const geometry_msgs::PoseStamped& pose_est)
{

        geometry_msgs::TransformStamped transform;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tf2_listener(tfBuffer);
        transform = tfBuffer.lookupTransform(world_frame, pose_est.header.frame_id, ros::Time(0), ros::Duration(1.0));         //

        geometry_msgs::PoseStamped pose;
        tf2::doTransform(pose_est, pose, transform);

        gtsam::Point3 point(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
	std::cout<<"point from pose msg to gtsam "<<pose.pose.position.x<<"  "<<pose.pose.position.y<<"  "<<pose.pose.position.z<<"\n";

        gtsam::Rot3 rot = gtsam::Rot3::quaternion( pose.pose.orientation.w,
                                                   pose.pose.orientation.x,
                                                   pose.pose.orientation.y,
                                                   pose.pose.orientation.z);

        return gtsam::Pose3(rot, point);
}

unsigned int FlowerMapper::get_flower_index(const gtsam::Pose3& pose_est)
{
        // If this is the first flower observed, then create map
        if (map_.size() == 0)
        {
                int flower_index = ++flower_count_;
                map_.push_back(std::make_tuple(flower_index, pose_est.translation(), normal_));
                return flower_index;
        }
        else
        {
                // Look at all flowers in map to search for a match
                for (int i=0; i<map_.size(); i++)
                {
                        unsigned int current_flower_index = std::get<0>(map_[i]);
                        gtsam::Point3 current_position = std::get<1>(map_[i]);

                        gtsam::Point3 test_position = pose_est.translation();

                        double dist = current_position.distance(test_position);
                        if (dist <= association_distance_thresh)
                        {
                                auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [current_flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double, sensor_msgs::PointCloud2>&g) {return std::get<0>(g) == current_flower_index; });
                                if ( graph_iter != graphs_.end() )
                                {
                                        ++std::get<4>(*graph_iter);
                                }
                                return current_flower_index;

                        }
                }
                // If newly observed flower, then add to map
                unsigned int flower_index = ++flower_count_;
                map_.push_back(std::make_tuple(flower_index, pose_est.translation(), normal_));
                return flower_index;
        }
}


// calcuate point clouds normal vector in the world frame using plane fitting.
gtsam::Point3 FlowerMapper::get_pc_norm_vec(sensor_msgs::PointCloud2 pc)
{
        srand (0);

        tf_listener_pc_.waitForTransform(world_frame,pc.header.frame_id,ros::Time(0),ros::Duration(1.0));
        tf::StampedTransform transform;
        tf_listener_pc_.lookupTransform(world_frame,pc.header.frame_id, ros::Time(0), transform);


        // transform point cloud from camera to base
        sensor_msgs::PointCloud2 pc_transformed;
        Eigen::Matrix4f eigen_transform;
        pcl_ros::transformAsMatrix (transform, eigen_transform);
        pcl_ros::transformPointCloud(eigen_transform, pc, pc_transformed);


        PointCloud<PointXYZ>::Ptr cloud_ (new PointCloud<PointXYZ> ());
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(pc_transformed,pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud_);

        PointCloud<PointXYZ>::Ptr cloud_filtered_ (new PointCloud<PointXYZ> ());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered_);

        // Create a shared plane model pointer directly
        SampleConsensusModelPlanePtr model (new SampleConsensusModelPlane<PointXYZ> (cloud_filtered_));
        bool result;
        std::vector<int> inliers;
        Eigen::VectorXf coeff;
        switch (estimator_type)
        {
        case 0:
        {
                // LMeDs
                // Fusiello, Andrea. "Elements of geometric computer vision." Available fro m: http://homepages. inf. ed. ac. uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO4/tutorial. html (2006).
                // See Section 7.3.3
                LeastMedianSquares<PointXYZ> sac (model, model_distance_thresh);
                result = sac.computeModel ();

                sac.getInliers (inliers);
                sac.getModelCoefficients (coeff);

                break;
        }
        case 1:
        {
                // RRAANSAC
                // Chum, Ondrej, and Jirı Matas. "Randomized RANSAC with Td, d test." Proc. British Machine Vision Conference. Vol. 2. 2002.
                RandomizedRandomSampleConsensus<PointXYZ> sac (model, model_distance_thresh);
                result = sac.computeModel ();

                sac.getInliers (inliers);
                sac.getModelCoefficients (coeff);

                break;
        }
        case 2:
        {
                // MLESAC
                // Torr, Philip HS, and Andrew Zisserman. "MLESAC: A new robust estimator with application to estimating image geometry." Computer vision and image understanding 78.1 (2000): 138-156.
                MaximumLikelihoodSampleConsensus<PointXYZ> sac (model, model_distance_thresh);
                result = sac.computeModel ();

                sac.getInliers (inliers);
                sac.getModelCoefficients (coeff);

                break;
        }
        default:
        {
                // LMeDs
                // Fusiello, Andrea. "Elements of geometric computer vision." Available fro m: http://homepages. inf. ed. ac. uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO4/tutorial. html (2006).
                // See Section 7.3.3
                LeastMedianSquares<PointXYZ> sac (model, model_distance_thresh);
                result = sac.computeModel ();

                sac.getInliers (inliers);
                sac.getModelCoefficients (coeff);

                break;
        }
        }

        if (coeff.size() != 4)
          return gtsam::Point3(0.0,0.0,0.0);

        Eigen::VectorXf coeff_refined;
        model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
        coeff_refined(2) = -1.0*coeff_refined(2);

        Eigen::Vector3d plane_normal = (Eigen::Vector3d() << coeff_refined(0), coeff_refined(1), coeff_refined(2)).finished();

        Eigen::Vector4d centroid;
        compute3DCentroid(*cloud_, centroid);

        // Verify direction of normal vec (i.e., always point norm vec away from camera)
        Eigen::Vector3d camera_normal = (centroid.head(3)) / centroid.head(3).norm();
        if (plane_normal.dot(camera_normal) < 0.0) { plane_normal = -1.0 * plane_normal; }

        return gtsam::Point3(plane_normal(0), plane_normal(1), plane_normal(2));

}


void FlowerMapper::update_graph(const unsigned int& flower_index, const gtsam::Point3 flower_position_est, const gtsam::Point3 flower_unit_vector, const double& flower_prob)
{

        using gtsam::symbol_shorthand::X;

        auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double, sensor_msgs::PointCloud2>&g) {return std::get<0>(g) == flower_index; });

        auto map_iter = std::find_if(map_.begin(), map_.end(), [flower_index] (const std::tuple<unsigned int, gtsam::Point3, gtsam::Point3>&g) {return std::get<0>(g) == flower_index; });


        if ( graph_iter == graphs_.end() )
        {
                unsigned int key = 0;
                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<  noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                gtsam::Values values;
                gtsam::NonlinearFactorGraph graph;

                values.insert(X(key), flower_position_est);

                // add prior factor from classifier
                graph.add(PriorFactor<gtsam::Point3>(X(key), flower_position_est, measurement_noise));

                graphs_.push_back( std::make_tuple(flower_index, key, graph, values, 1, flower_prob, pc_ ));
                return;
        }
        else
        {
                gtsam::Values values;
                gtsam::NonlinearFactorGraph graph;

                ++std::get<1>(*graph_iter);

                diagNoise::shared_ptr measurement_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<    noise_position_classifier, noise_position_classifier, noise_position_classifier ).finished());

                diagNoise::shared_ptr process_noise = diagNoise::Sigmas(( gtsam::Vector(3) <<   noise_position_process, noise_position_process, noise_position_process ).finished());

                unsigned int key = std::get<1>(*graph_iter);
                std::get<3>(*graph_iter).insert(X(key), flower_position_est);

                // Add prior from classifier
                (std::get<2>(*graph_iter)).add(PriorFactor<gtsam::Point3>(X(key), flower_position_est, measurement_noise));

                // Process noise update
                gtsam::Point3 process_est(gtsam::Point3(0, 0, 0));
                (std::get<2>(*graph_iter)).add(BetweenFactor<gtsam::Point3>( X(key), X(key-1), process_est, process_noise));

                // Optimizer graph
                std::get<3>(*graph_iter) = LevenbergMarquardtOptimizer(std::get<2>(*graph_iter), std::get<3>(*graph_iter)).optimize();


                gtsam::Point3 position_est = std::get<3>(*graph_iter).at<gtsam::Point3>(X(key));
                gtsam::Pose3 pose_est(gtsam::Rot3::RzRyRx(0,0,0), position_est);

                std::get<5>(*graph_iter)*=flower_prob;
                if ( map_iter == map_.end() )
                {
                        map_.push_back(std::make_tuple(flower_index, pose_est.translation(), normal_));
                }
                else
                {
                        std::get<1>(*map_iter) = pose_est.translation();
                }
        }
        return;

}


void FlowerMapper::send_flower_map(manipulation_common::UpdateFlowerMap::Response &res, const ros::Publisher& pub)
{
        manipulation_common::Flower flower;
        manipulation_common::FlowerMap flower_map;

        auto timestamp = ros::Time::now();
        for (int i=0; i<map_.size(); i++)
        {
                flower.id = std::get<0>(map_[i]);
                unsigned int flower_index = std::get<0>(map_[i]);
                gtsam::Point3 point_est = std::get<1>(map_[i]);
                gtsam::Point3 unit_vec = std::get<2>(map_[i]);

                auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double, sensor_msgs::PointCloud2>&g) {return std::get<0>(g) == flower_index; });

                if (std::get<4>(*graph_iter)>=observation_thresh)
                {

                        gtsam::Vector3 position = point_est;
                        gtsam::Vector3 normalVec = unit_vec;

                        flower.header.frame_id = world_frame;
                        flower.header.stamp = timestamp;

                        flower.num_obs = std::get<4>(*graph_iter);
                        flower.prob = std::get<5>(*graph_iter);


                        // Position 
                        flower.point.header.frame_id = world_frame;
                        flower.point.header.stamp = timestamp;
                        flower.point.point.x = position[0];
                        flower.point.point.y = position[1];
                        flower.point.point.z = position[2];

                        // Unit Vector added *10 Trevor
                        flower.vec.header.frame_id = world_frame;
                        flower.vec.header.stamp = timestamp;
                        flower.vec.vector.x = normalVec[0];
                        flower.vec.vector.y = normalVec[1];
                        flower.vec.vector.z = normalVec[2];


                        PointCloud<PointXYZ>::Ptr cloud_ (new PointCloud<PointXYZ> ());

                        pcl::PCLPointCloud2 pcl_pc2;
                        auto pc = std::get<6>(*graph_iter);
                        pcl_conversions::toPCL(pc,pcl_pc2);
                        pcl::fromPCLPointCloud2(pcl_pc2,*cloud_);

                        // Publish the pointcloud
                        // flower.point_cloud.header.seq = pcl_pc2.header.seq;
                        // flower.point_cloud.header.stamp = timestamp;
                        // flower.point_cloud.header.frame_id = pcl_pc2.header.frame_id;
                        //
                        // flower.point_cloud.data = pcl_pc2.data;
                        // flower.point_cloud.height = pcl_pc2.height;
                        // flower.point_cloud.width = pcl_pc2.width;
                        // flower.point_cloud.is_bigendian = pcl_pc2.is_bigendian;
                        // flower.point_cloud.point_step = pcl_pc2.point_step;
                        // flower.point_cloud.row_step = pcl_pc2.row_step;
                        // flower.point_cloud.is_dense = pcl_pc2.is_dense;

                        // DO SVD DECOMP -- Only for validation
                        auto const m = cloud_->getMatrixXfMap();

                        Eigen::RowVectorXf means = (m.transpose()).colwise().mean();
                        auto data_zm = (m.transpose()).rowwise() - means;

                        Eigen::JacobiSVD<Eigen::MatrixXf> svd( data_zm.transpose(), Eigen::ComputeFullU );
                        const Eigen::MatrixXf U = svd.matrixU();
                        const Eigen::ArrayXf S = svd.singularValues();

                        Eigen::Vector3f eigen_vals = S.head(3);
                        Eigen::Vector3f major_vec = U.col(0).head(3);
                        Eigen::Vector3f middle_vec = U.col(1).head(3);
                        Eigen::Vector3f minor_vec = U.col(2).head(3);

                        flower.eigen_decomp.eigen_values.x = eigen_vals[0];
                        flower.eigen_decomp.eigen_values.y = eigen_vals[1];
                        flower.eigen_decomp.eigen_values.z = eigen_vals[2];

                        flower.eigen_decomp.major_vec.x = major_vec[0];
                        flower.eigen_decomp.major_vec.y = major_vec[1];
                        flower.eigen_decomp.major_vec.z = major_vec[2];

                        flower.eigen_decomp.middle_vec.x = middle_vec[0];
                        flower.eigen_decomp.middle_vec.y = middle_vec[1];
                        flower.eigen_decomp.middle_vec.z = middle_vec[2];

                        flower.eigen_decomp.minor_vec.x = minor_vec[0];
                        flower.eigen_decomp.minor_vec.y = minor_vec[1];
                        flower.eigen_decomp.minor_vec.z = minor_vec[2];

                        flower_map.map.push_back(flower);
                        res.map.push_back(flower);
                }
        }
        if(pub.getNumSubscribers() != 0) { pub.publish(flower_map); }
        if(pub.getNumSubscribers() != 0) { pub.publish(flower_map); }

        flower_map.map.clear();
}

void FlowerMapper::send_flower_vec(manipulation_common::UpdateFlowerMap::Response &res, const ros::Publisher& pub)
{
        manipulation_common::Flower flower;
        manipulation_common::FlowerMap flower_map;
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markerArray;
        std::vector<sensor_msgs::PointCloud2> point_clouds; point_clouds.clear();

        auto timestamp = ros::Time::now();
        for (int i=0; i<map_.size(); i++)
        {
                marker.id = std::get<0>(map_[i]);
                unsigned int flower_index = std::get<0>(map_[i]);
                gtsam::Point3 point_est = std::get<1>(map_[i]);
                gtsam::Point3 unit_vec = std::get<2>(map_[i]);

                auto graph_iter = std::find_if(graphs_.begin(), graphs_.end(), [flower_index] (const std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double, sensor_msgs::PointCloud2>&g) {return std::get<0>(g) == flower_index; });

                if (std::get<4>(*graph_iter)>=observation_thresh)
                {
                        auto pc = std::get<6>(*graph_iter);
                        point_clouds.push_back(pc);

                        gtsam::Vector3 position = point_est;
                        gtsam::Vector3 normalVec = unit_vec;

                        flower.header.frame_id = world_frame;
                        flower.header.stamp = timestamp;

                        flower.num_obs = std::get<4>(*graph_iter);
                        flower.prob = std::get<5>(*graph_iter);

                        // Position added
                        flower.point.header.frame_id = world_frame;
                        flower.point.header.stamp = timestamp;
                        flower.point.point.x = position[0];
                        flower.point.point.y = position[1];
                        flower.point.point.z = position[2];

                        // Unit Vector added
                        flower.vec.header.frame_id = world_frame;
                        flower.vec.header.stamp = timestamp;
                        flower.vec.vector.x = normalVec[0];
                        flower.vec.vector.y = normalVec[1];
                        flower.vec.vector.z = normalVec[2];

                        marker.scale.x= 0.005;
                        marker.scale.y= 0.009;

                        marker.header.frame_id = world_frame;
                        marker.header.stamp = timestamp;
                        marker.type = visualization_msgs::Marker::ARROW;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.points.push_back(flower.point.point);


                        geometry_msgs::Point end;
                        end.x = flower.point.point.x - flower.vec.vector.x/20;
                        end.y = flower.point.point.y - flower.vec.vector.y/20;
                        end.z = flower.point.point.z - flower.vec.vector.z/20;
                        marker.points.push_back(end);

                        marker.color.r = 1.0f;
                        marker.color.g = 0.0f;
                        marker.color.b = 0.0f;
                        marker.color.a = 1.0;

                        markerArray.markers.push_back(marker);
                        marker.points.clear();

                        std::cout << "pts, markers = " << point_clouds.size() << ", " << markerArray.markers.size() << std::endl;
                }
        }
    // if(pub.getNumSubscribers() != 0) { pub.publish(markerArray); }
    pub.publish(markerArray);
    markerArray.markers.clear();

    if(point_clouds.size()>0) { publish_point_clouds(point_clouds); }
}

void FlowerMapper::publish_point_clouds(std::vector<sensor_msgs::PointCloud2> points_clouds)
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_merged_pc;
  for(int i = 0; i < points_clouds.size(); i++) //for all points in segment
  {
    //convert ith sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::moveToPCL(points_clouds[i], pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::fromPCLPointCloud2(pcl_pc2,pc);

    //loop over points
    for(int j = 0; j < pc.size(); j++)
    {
      pcl_merged_pc.push_back(pc.at(j));
    }
  }

  //convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs::PointCloud2
  pcl::PCLPointCloud2 pcl_merged_pc2;
  pcl::toPCLPointCloud2(pcl_merged_pc, pcl_merged_pc2);
  sensor_msgs::PointCloud2 ros_merged_pc2;
  pcl_conversions::moveFromPCL(pcl_merged_pc2, ros_merged_pc2);
  ros_merged_pc2.header = points_clouds[0].header;

  //publish merged point clouds
  pts_pub_.publish(ros_merged_pc2);
}

bool FlowerMapper::load_parameters(const ros::NodeHandle& nh){

        // // Load filter noise specs
        if(!pu::Get("filter/noise_position_classifier", noise_position_classifier)) return false;
        if(!pu::Get("filter/noise_attitude_classifier", noise_attitude_classifier)) return false;
        if(!pu::Get("filter/noise_position_process", noise_position_process)) return false;
        if(!pu::Get("filter/noise_attitude_process", noise_attitude_process)) return false;
        if(!pu::Get("filter/association_distance_thresh", association_distance_thresh)) return false;
        if(!pu::Get("filter/position_only", position_only)) return false;
        if(!pu::Get("filter/make_robust", make_robust)) return false;
        if(!pu::Get("filter/world_frame", world_frame)) return false;
        if(!pu::Get("filter/observation_thresh", observation_thresh)) return false;

        if(!pu::Get("attitude_est/model_distance_thresh", model_distance_thresh)) return false;
        if(!pu::Get("attitude_est/estimator_type", estimator_type)) return false;

        return true;
}
///////////////////////
// End Helper Methods//
///////////////////////



} // End Namespace
