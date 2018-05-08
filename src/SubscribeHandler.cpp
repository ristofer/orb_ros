/**
* This file is part of the odometry integration of pepper in SLAM
* Author: Tim Resink
* Email: timresink@gmail.com
*/
#include"SubscribeHandler.hpp"

using namespace Eigen;



SubscribeHandler::SubscribeHandler(const string &strVocFile,
                                   const string &strSettingsFile, ros::NodeHandle* pNodeHandler):
mpNodeHandler(pNodeHandler),
mbReferenceWorldFrame(false)
{
      cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
      cameraTopic = (std::string) fsSettings["Topic.Camera"];
      cameraFrameTopic = (std::string) fsSettings["Topic.CameraFrame"];
      odomFrameTopic = (std::string) fsSettings["Topic.OdomFrame"];
      tfTopic =(std::string) fsSettings["Topic.TF"];
      int queueSize = (int) fsSettings["Topic.QueueSize"];
      baseFrameTopic = (std::string) fsSettings["Topic.BaseFrame"];
      useBaseFrame = (int) fsSettings["Initializer.baseFrame"];
      cameraFrameNameToPublish = (std::string) fsSettings["Topic.CameraFrameNameToPublish"];
      worldFrameNameToPublish = (std::string) fsSettings["Topic.WorldFrameNameToPublish"];

      mpSLAM = new ORB_SLAM2::System(strVocFile, strSettingsFile, ORB_SLAM2::System::MONOCULAR,true,true);

      if(useBaseFrame)
          broadCastTopic = baseFrameTopic + "_ORB";
      else
          broadCastTopic = cameraFrameNameToPublish;

      subImage = mpNodeHandler->subscribe(cameraTopic, 1, &SubscribeHandler::GrabImage, this);
      maqui_orientation = mpNodeHandler->advertise<geometry_msgs::PoseStamped>("/maqui/odom_ORB", queueSize);
      tracking_state = mpNodeHandler->advertise<std_msgs::Int8>("/orb_slam_tracking_state", queueSize);
      current_frame = mpNodeHandler->advertise<sensor_msgs::Image>("/orb_slam_current_frame", queueSize);

      // Initialize ORB system
   // argument 4 boolean is user viewer
}



void SubscribeHandler::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Twc = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());


    int TrackingState = mpSLAM->GetTrackingState();
    SubscribeHandler::Publish_Tracking_State(TrackingState);
    cv::Mat im = mpSLAM->DrawFrame();
    cv_bridge::CvImage current_frame_msg;
    current_frame_msg.header = msg->header;
    current_frame_msg.encoding = msg->encoding;
    current_frame_msg.image = im;
    current_frame.publish(current_frame_msg.toImageMsg());

}




void SubscribeHandler::Publish_Tracking_State(int state)
{
    std_msgs::Int8 StateMsg;
    StateMsg.data = state;
    tracking_state.publish(StateMsg);
    return;
}


g2o::SE3Quat SubscribeHandler::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat SubscribeHandler::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=eigMat(i,j);

    return cvMat.clone();
}

Eigen::Matrix<double,3,3> SubscribeHandler::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;
    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> SubscribeHandler::toQuaternion(const Eigen::Matrix<double, 3, 3> &M)
{
    Eigen::Quaterniond q(M);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    std::vector<float> quat = SubscribeHandler::Normalize(v);
    return quat;
//      return v;
}


std::vector<float> SubscribeHandler::Normalize(std::vector<float> vect)
{
    float sum = 0;
    // normalize
    for (unsigned int i = 0; i == vect.size(); i++)
    {
        sum+= vect[i]*vect[i];
    }

    float scalar = 1/std::sqrt(sum);

    for (unsigned int j = 0; j == vect.size(); j++)
    {
        vect[j] *= scalar;
    }
    return vect;
}

void SubscribeHandler::Shutdown(){

	std::cout << "ROS shutdown" << std::endl;
	ros::shutdown();
    mpSLAM->Shutdown();
}



