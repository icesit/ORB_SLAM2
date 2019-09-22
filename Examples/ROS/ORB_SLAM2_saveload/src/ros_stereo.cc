/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/CompressedImage.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM)
    , firstpub(true)
    {
        slamPos = nh.advertise<geometry_msgs::PoseStamped>("/slam/pose",5);
        slamPosWithcovar = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam/posecov",5);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    void GrabCompressedStereo(const sensor_msgs::CompressedImageConstPtr& msgLeft,const sensor_msgs::CompressedImageConstPtr& msgRight);
    void pubpose(ros::Time rost);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    ros::NodeHandle nh;
    ros::Publisher slamPos, slamPosWithcovar;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::PoseWithCovarianceStamped lastmsg;
    geometry_msgs::PoseStamped lastmsgnocov;
    bool firstpub;
};

void myspin(ros::NodeHandle &nh, ORB_SLAM2::System &SLAM){
    // ros::Publisher slamPos = nh.advertise<geometry_msgs::PoseStamped>("/slam/pose",10);
    /*ros::Publisher slamPos = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam/pose",10);
    geometry_msgs::PoseStamped msg,lastmsg;
    geometry_msgs::PoseWithCovarianceStamped msgcov,lastmsgcov;
    bool firstpub=true;
    cv::Mat Twc(3,1,CV_32F);
    float q[4];
    ros::Rate r(20);
    while(ros::ok()){
        if(SLAM.GetFramePose(Twc, q)){
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = Twc.at<float>(2);//Twc.at<float>(0);
        msg.pose.pose.position.y = -Twc.at<float>(0);//Twc.at<float>(1);
        msg.pose.pose.position.z = -Twc.at<float>(1);//Twc.at<float>(2);
        msg.pose.pose.orientation.x = q[2];//q[0];
        msg.pose.pose.orientation.y = -q[0];//q[1];
        msg.pose.pose.orientation.z = -q[1];//q[2];
        msg.pose.pose.orientation.w = q[3];
        if(firstpub){
            firstpub = false;
        }
        else{
            msg.pose.covariance[0] = abs(msg.pose.pose.position.x - lastmsg.pose.pose.position.x)/100;
            msg.pose.covariance[7] = abs(msg.pose.pose.position.y - lastmsg.pose.pose.position.y)/100;
            msg.pose.covariance[14] = abs(msg.pose.pose.position.z - lastmsg.pose.pose.position.z)/100;
            msg.pose.covariance[21] = 0;
            msg.pose.covariance[28] = 0;
            msg.pose.covariance[35] = 0;
        }
        slamPos.publish(msg);
        lastmsg = msg;
        ros::spinOnce();
        r.sleep();
        }
    }*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo");
    ros::start();

    if(argc != 7)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify save_map_ornot map_path" << endl;
        ros::shutdown();
        return 1;
    }    
    bool isdisplay;
    stringstream isd(argv[5]);
    isd >> boolalpha >> isdisplay;
    bool issavemap = (bool)atoi(argv[4]);
    string save_path = argv[6];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,isdisplay,issavemap, save_path+"map.bin");

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    bool compressimage;
    nh.param("/orb_slam_node/use_compressed", compressimage, false);

    if(compressimage){
    message_filters::Subscriber<sensor_msgs::CompressedImage> left_sub(nh, "/sensors/stereo_cam/left/image_rect_color/compressed", 1);
    message_filters::Subscriber<sensor_msgs::CompressedImage> right_sub(nh, "/sensors/stereo_cam/right/image_rect_color/compressed", 1);
    //message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/usb_cam0/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/usb_cam1/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabCompressedStereo,&igb,_1,_2));
    cout<<"compress!!!!!!!!!!"<<endl;
//    myspin(nh, SLAM);
    ros::spin();
    }
    else{
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/sensors/stereo_cam/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/sensors/stereo_cam/right/image_rect_color", 1);
    //message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/usb_cam0/image_raw", 1);
    //message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/usb_cam1/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    cout<<"no compress!!!!!!!!!!"<<endl;
//    myspin(nh, SLAM);
    ros::spin();
    }
    
    // Stop all threads
    SLAM.Shutdown();

    if(issavemap){
    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM(save_path+"KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(save_path+"Input/trajectory.txt");
    SLAM.SaveMappointPos(save_path+"Input/MapPointsPos.txt");
    //SLAM.SaveTrajectoryTUM(save_path+"FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI(save_path+"FrameTrajectory_KITTI_Format.txt");
	}

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    ros::Time rost = msgLeft->header.stamp;//ros::Time::now();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    pubpose(rost);

}

void ImageGrabber::GrabCompressedStereo(const sensor_msgs::CompressedImageConstPtr& msgLeft,const sensor_msgs::CompressedImageConstPtr& msgRight)
{
    ros::Time rost = msgLeft->header.stamp;//ros::Time::now();
    cv::Mat imLeft_input = cv::imdecode(cv::Mat(msgLeft->data),1);
    cv::Mat imRight_input = cv::imdecode(cv::Mat(msgRight->data),1);
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(imLeft_input,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight_input,imRight,M1r,M2r,cv::INTER_LINEAR);
        mpSLAM->TrackStereo(imLeft,imRight,msgLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(imLeft_input,imRight_input,msgLeft->header.stamp.toSec());
    }
    pubpose(rost);
}

void ImageGrabber::pubpose(ros::Time rost){
    // geometry_msgs::PoseStamped msg;
    geometry_msgs::PoseWithCovarianceStamped msg;
    geometry_msgs::PoseStamped msgnocov;
    geometry_msgs::TransformStamped transformStamped;
    cv::Mat Twc(3,1,CV_32F);
    float q[4];
    if(mpSLAM->GetFramePose(Twc, q)){
        msg.header.stamp = rost;
        msg.header.frame_id = "/slam";
        msg.pose.pose.position.x = Twc.at<float>(2);//Twc.at<float>(0);
        msg.pose.pose.position.y = -Twc.at<float>(0);//Twc.at<float>(1);
        msg.pose.pose.position.z = -Twc.at<float>(1);//Twc.at<float>(2);
        msg.pose.pose.orientation.x = q[2];//q[0];
        msg.pose.pose.orientation.y = -q[0];//q[1];
        msg.pose.pose.orientation.z = -q[1];//q[2];
        msg.pose.pose.orientation.w = q[3];

        msgnocov.header.stamp = rost;
        msgnocov.header.frame_id = "/slam";
        msgnocov.pose.position.x = Twc.at<float>(2);//Twc.at<float>(0);
        msgnocov.pose.position.y = -Twc.at<float>(0);//Twc.at<float>(1);
        msgnocov.pose.position.z = -Twc.at<float>(1);//Twc.at<float>(2);
        msgnocov.pose.orientation.x = q[2];//q[0];
        msgnocov.pose.orientation.y = -q[0];//q[1];
        msgnocov.pose.orientation.z = -q[1];//q[2];
        msgnocov.pose.orientation.w = q[3];
        if(firstpub){
            firstpub = false;
        }
        else{
            msg.pose.covariance[0] = abs(msg.pose.pose.position.x - lastmsg.pose.pose.position.x)/100;
            msg.pose.covariance[7] = abs(msg.pose.pose.position.y - lastmsg.pose.pose.position.y)/100;
            msg.pose.covariance[14] = abs(msg.pose.pose.position.z - lastmsg.pose.pose.position.z)/100;
            msg.pose.covariance[21] = 0.001;
            msg.pose.covariance[28] = 0.001;
            msg.pose.covariance[35] = 0.001;
        }
        slamPos.publish(msgnocov);
        slamPosWithcovar.publish(msg);
        lastmsg = msg;
        lastmsgnocov = msgnocov;

        transformStamped.header.stamp = rost;
        transformStamped.header.frame_id = "/slam";
        transformStamped.child_frame_id = "/camera";
        transformStamped.transform.translation.x = msg.pose.pose.position.x;
        transformStamped.transform.translation.y = msg.pose.pose.position.y;
        transformStamped.transform.translation.z = msg.pose.pose.position.z;
        transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;
        br.sendTransform(transformStamped);
    }
}
