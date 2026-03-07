/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "System.h"
#include "Converter.h"
#include <thread>
// #include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <cv_bridge/cv_bridge.h>

namespace ORB_SLAM3
{
  Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

  template <typename T>
  double GetTimeStampFromMsg(const T &msg)
  {
    return msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
  }

  cv::Mat System::GetImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
  {
    cv_bridge::CvImageConstPtr ptr;
    try
    {
      ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
    cv::Mat img = ptr->image.clone();
    return img;
  }

  System::System(std::string node_name) : Node(node_name), mbReset(false), mbResetActiveMap(false),
                                          mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
  {
    mStrVocabularyFilePath = this->declare_parameter<std::string>("vocabulary_file_path");
    const std::string strSettingsFile = this->declare_parameter<std::string>("settings_file_path");
    mSensor = static_cast<System::eSensor>(this->declare_parameter<int>("sensor_type"));
    const int initFr = this->declare_parameter<int>("init_fr", 0);
    const std::string strSequence = this->declare_parameter<std::string>("sequence_name", "");
    const bool enableLoopClosing = this->declare_parameter<bool>("enable_loop", true);

    cout << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
      cout << "Monocular" << endl;
    else if (mSensor == STEREO)
      cout << "Stereo" << endl;
    else if (mSensor == RGBD)
      cout << "RGB-D" << endl;
    else if (mSensor == IMU_MONOCULAR)
      cout << "Monocular-Inertial" << endl;
    else if (mSensor == IMU_STEREO)
      cout << "Stereo-Inertial" << endl;
    else if (mSensor == IMU_RGBD)
      cout << "RGB-D-Inertial" << endl;

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
      cerr << "Failed to open settings file at: " << strSettingsFile << endl;
      exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
      settings_ = new Settings(strSettingsFile, mSensor);

      mStrLoadAtlasFromFile = settings_->atlasLoadFile();
      mStrSaveAtlasToFile = settings_->atlasSaveFile();

      cout << (*settings_) << endl;
    }
    else
    {
      settings_ = nullptr;
      cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
      if (!node.empty() && node.isString())
      {
        mStrLoadAtlasFromFile = (string)node;
      }

      node = fsSettings["System.SaveAtlasToFile"];
      if (!node.empty() && node.isString())
      {
        mStrSaveAtlasToFile = (string)node;
      }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if (!node.empty())
    {
      activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    // mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if (mStrLoadAtlasFromFile.empty())
    {
      // Load ORB Vocabulary
      cout << endl
           << "Loading ORB Vocabulary. This could take a while..." << endl;

      mpVocabulary = new ORBVocabulary();
      bool bVocLoad = mpVocabulary->loadFromBinaryFile(mStrVocabularyFilePath);
      if (!bVocLoad)
      {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << mStrVocabularyFilePath << endl;
        exit(-1);
      }
      cout << "Vocabulary loaded!" << endl
           << endl;

      // Create KeyFrame Database
      mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

      // Create the Atlas
      cout << "Initialization of Atlas from scratch " << endl;
      mpAtlas = new Atlas(0);
    }
    else
    {
      // Load ORB Vocabulary
      cout << endl
           << "Loading ORB Vocabulary. This could take a while..." << endl;

      mpVocabulary = new ORBVocabulary();
      bool bVocLoad = mpVocabulary->loadFromTextFile(mStrVocabularyFilePath);
      if (!bVocLoad)
      {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << mStrVocabularyFilePath << endl;
        exit(-1);
      }
      cout << "Vocabulary loaded!" << endl
           << endl;

      // Create KeyFrame Database
      mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

      cout << "Load File" << endl;

      // Load the file with an earlier session
      // clock_t start = clock();
      cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
      bool isRead = LoadAtlas(FileType::BINARY_FILE);

      if (!isRead)
      {
        cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
        exit(-1);
      }
      // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

      // cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

      loadedAtlas = true;

      mpAtlas->CreateNewMap();

      // clock_t timeElapsed = clock() - start;
      // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
      // cout << "Binary file read in " << msElapsed << " ms" << endl;

      // usleep(10*1000*1000);
    }

    if (mSensor == IMU_STEREO || mSensor == IMU_MONOCULAR || mSensor == IMU_RGBD)
      mpAtlas->SetInertialSensor();

    // Create Drawers. These are used by the Viewer
    //  mpFrameDrawer = new FrameDrawer(mpAtlas);
    //  mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    // Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, mpVocabulary,
                             mpAtlas, mpKeyFrameDatabase,
                             strSettingsFile, mSensor, settings_, strSequence);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor == MONOCULAR || mSensor == IMU_MONOCULAR,
                                     mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if (settings_)
      mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
      mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if (mpLocalMapper->mThFarPoints != 0)
    {
      cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
      mpLocalMapper->mbFarPoints = true;
    }
    else
      mpLocalMapper->mbFarPoints = false;

    // Initialize the Loop Closing thread and launch
    //  mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // usleep(10*1000*1000);

    // Initialize the Viewer thread and launch
    //  if(bUseViewer)
    //  //if(false) // TODO
    //  {
    //      mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
    //      mptViewer = new thread(&Viewer::Run, mpViewer);
    //      mpTracker->SetViewer(mpViewer);
    //      mpLoopCloser->mpViewer = mpViewer;
    //      mpViewer->both = mpFrameDrawer->both;
    //  }

    // Fix verbosity
    std::cout << "system initialized!" << std::endl;
    InitializeRos();
    std::cout << "ros initialized!" << std::endl;
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);
  }

  void System::InitializeRos()
  {
    mImageSub = this->create_subscription<sensor_msgs::msg::Image>("image", 100, std::bind(&System::ImgCallback, this, std::placeholders::_1));
    if (mSensor == eSensor::RGBD || mSensor == eSensor::IMU_RGBD)
      mDepthSub = this->create_subscription<sensor_msgs::msg::Image>("depth", 100, std::bind(&System::DepthCallback, this, std::placeholders::_1));

    if (mSensor == eSensor::IMU_RGBD || mSensor == eSensor::IMU_MONOCULAR || mSensor == eSensor::IMU_STEREO)
      mImuSub = this->create_subscription<sensor_msgs::msg::Imu>("imu", 200, std::bind(&System::IMUCallback, this, std::placeholders::_1));

    // mPosePublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    mMapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);
    mLocalMapPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 100);
    mKeyFramePublisher = this->create_publisher<geometry_msgs::msg::PoseArray>("keyframes", 100);
    mCovizGraphPublisher = this->create_publisher<visualization_msgs::msg::Marker>("coviz_graph", 100);
    mOdometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 100);

    mPosePubTimer = this->create_wall_timer(30ms, std::bind(&System::PublishOdometry, this));
    mMapPubTimer = this->create_wall_timer(100ms, std::bind(&System::PublishMap, this));

    mTrackingTimer = this->create_wall_timer(2ms, std::bind(&System::SyncImageAndDepth, this));
  }

  void System::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
  {
    double t = GetTimeStampFromMsg(imu_msg);
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    auto IMU_data = IMU::Point(dx, dy, dz, rx, ry, rz, t);

    std::unique_lock<mutex> lock(mMutexBuffer);
    mQueueImuBuffer.push_back(IMU_data);
    return;
  }

  void System::ImgCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
  {
    std::unique_lock<mutex> lock(mMutexBuffer);
    mImageBuffer.push(img_msg);
  }

  void System::DepthCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
  {
    std::unique_lock<mutex> lock(mMutexBuffer);
    mDepthBuffer.push(img_msg);
  }

  void System::SyncImageAndDepth()
  {
    auto t1 = std::chrono::system_clock().now();

    cv::Mat image0, image1;
    std_msgs::msg::Header header;
    double time = 0;
    {
      std::unique_lock<mutex> lock(mMutexBuffer);
      if (!mImageBuffer.empty() && !mDepthBuffer.empty())
      {
        double time0 = GetTimeStampFromMsg(mImageBuffer.front());
        double time1 = GetTimeStampFromMsg(mDepthBuffer.front());
        // 0.003s sync tolerance
        if (time0 < time1 - 0.003)
        {
          mImageBuffer.pop();
          RCLCPP_DEBUG(this->get_logger(), "Throw color image");
        }
        else if (time0 > time1 + 0.003)
        {
          mDepthBuffer.pop();
          RCLCPP_DEBUG(this->get_logger(), "Throw depth image");
        }
        else
        {
          time = mImageBuffer.front()->header.stamp.sec + mImageBuffer.front()->header.stamp.nanosec * (1e-9);
          header = mImageBuffer.front()->header;
          image0 = GetImageFromMsg(mImageBuffer.front());
          mImageBuffer.pop();
          image1 = GetImageFromMsg(mDepthBuffer.front());
          mDepthBuffer.pop();
          RCLCPP_DEBUG(this->get_logger(), "found valid pair");
        }
      }

      if (!image0.empty())
      {
        std::vector<IMU::Point> ImuBufferCopy;
        if (mSensor == eSensor::IMU_RGBD)
        {
          if (mQueueImuBuffer.empty())
            return;

          std::copy(mQueueImuBuffer.begin(), mQueueImuBuffer.end(), std::back_inserter(ImuBufferCopy));
          mQueueImuBuffer.clear();
        }

        this->TrackRGBD(image0, image1, time, ImuBufferCopy);
      }
    }

    auto t2 = std::chrono::system_clock().now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  }

  void System::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
  {
    std::unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
  }

  void System::PublishOdometry()
  {
    Eigen::Matrix4f Twc;
    {
      std::unique_lock<mutex> lock(mMutexCamera);
      Twc = mCameraPose.matrix();
    }

    // Eigen::Matrix4f cv2gl;
    // cv2gl << 1, 0, 0, 0,
    //          0, 1, 0, 0,
    //          0, 0, 1, 0,
    //          0, 0, 0, 1;
    // Eigen::Matrix4f Twc_gl = cv2gl * Twc;

    const float PI = 3.141592;
    Eigen::AngleAxisf yawAngle(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(-PI/2, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf qr = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3f rotationMatrix = qr.matrix();
    Eigen::Matrix4f cv2gl = Eigen::Matrix4f::Identity();
    cv2gl.topLeftCorner<3, 3>() = rotationMatrix;
    Twc = cv2gl * Twc;

    // convert to quaternion
    Eigen::Quaternionf q(Twc.topLeftCorner<3, 3>());
    q.normalize();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = this->get_clock()->now();
    // odom_msg.child_frame_id = "map";
    odom_msg.pose.pose.position.x = Twc(0, 3);
    odom_msg.pose.pose.position.y = Twc(1, 3);
    odom_msg.pose.pose.position.z = Twc(2, 3);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    mOdometryPublisher->publish(odom_msg);
  }

  void System::PublishPose()
  {
    Eigen::Matrix4f Twc;
    {
      std::unique_lock<mutex> lock(mMutexCamera);
      Twc = mCameraPose.matrix();
    }

    // convert to quaternion
    Eigen::Quaternionf q(Twc.topLeftCorner<3, 3>());
    q.normalize();

    // publish
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.pose.position.x = Twc(0, 3);
    pose_msg.pose.position.y = Twc(1, 3);
    pose_msg.pose.position.z = Twc(2, 3);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    mPosePublisher->publish(pose_msg);
  }

  void System::PublishMap()
  {
    Map *pActiveMap = mpAtlas->GetCurrentMap();
    if (!pActiveMap)
      return;

    auto timestamp = this->get_clock()->now();

    const std::vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    auto point_size = vpRefMPs.size();
    float *points_ref = new float[point_size * 3];
    std::set<int> ref_ids;
    for (size_t i = 0; i < vpRefMPs.size(); i++)
    {
      MapPoint *pMP = vpRefMPs[i];
      if (pMP)
      {
        geometry_msgs::msg::Point pt;
        auto eig_pt = pMP->GetWorldPos();
        points_ref[i * 3 + 0] = eig_pt.x();
        points_ref[i * 3 + 1] = eig_pt.z();
        points_ref[i * 3 + 2] = -eig_pt.y();
        ref_ids.insert(pMP->mnId);
      }
    }

    sensor_msgs::msg::PointCloud2 point_cloud_ref;
    point_cloud_ref.header.frame_id = "world";
    point_cloud_ref.header.stamp = timestamp;
    point_cloud_ref.height = 1;
    point_cloud_ref.width = point_size;
    point_cloud_ref.fields.resize(3);
    point_cloud_ref.fields[0].name = "x";
    point_cloud_ref.fields[0].offset = 0;
    point_cloud_ref.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_ref.fields[0].count = 1;
    point_cloud_ref.fields[1].name = "y";
    point_cloud_ref.fields[1].offset = 4;
    point_cloud_ref.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_ref.fields[1].count = 1;
    point_cloud_ref.fields[2].name = "z";
    point_cloud_ref.fields[2].offset = 8;
    point_cloud_ref.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud_ref.fields[2].count = 1;
    point_cloud_ref.is_bigendian = false;
    point_cloud_ref.point_step = 12;
    point_cloud_ref.row_step = point_cloud_ref.point_step * point_cloud_ref.width;
    point_cloud_ref.is_dense = false;
    point_cloud_ref.data.resize(point_cloud_ref.row_step * point_cloud_ref.height);
    memcpy(point_cloud_ref.data.data(), points_ref, point_size * 3 * sizeof(float));

    mLocalMapPublisher->publish(point_cloud_ref);

    const std::vector<MapPoint *> &vpMPs = pActiveMap->GetAllMapPoints();

    if (vpMPs.empty())
      return;

    point_size = vpMPs.size();
    float *points = new float[point_size * 3];
    int point_count = 0;

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
      MapPoint *pMP = vpMPs[i];
      if (pMP && ref_ids.count(pMP->mnId) == 0)
      {
        auto eig_pt = pMP->GetWorldPos();
        points[point_count * 3 + 0] = eig_pt.x();
        points[point_count * 3 + 1] = eig_pt.z();
        points[point_count * 3 + 2] = -eig_pt.y();
        point_count++;
      }
    }

    sensor_msgs::msg::PointCloud2 point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = timestamp;
    point_cloud.height = 1;
    point_cloud.width = point_count;
    point_cloud.fields.resize(3);
    point_cloud.fields[0].name = "x";
    point_cloud.fields[0].offset = 0;
    point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud.fields[0].count = 1;
    point_cloud.fields[1].name = "y";
    point_cloud.fields[1].offset = 4;
    point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud.fields[1].count = 1;
    point_cloud.fields[2].name = "z";
    point_cloud.fields[2].offset = 8;
    point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    point_cloud.fields[2].count = 1;
    point_cloud.is_bigendian = false;
    point_cloud.point_step = 12;
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.is_dense = false;
    point_cloud.data.resize(point_cloud.row_step * point_cloud.height);
    memcpy(point_cloud.data.data(), points, point_count * 3 * sizeof(float));

    mMapPublisher->publish(point_cloud);

    delete[] points;
    delete[] points_ref;

    const std::vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

    geometry_msgs::msg::PoseArray keyframe_msg;
    keyframe_msg.header.frame_id = "world";
    keyframe_msg.header.stamp = this->get_clock()->now();
    keyframe_msg.poses.resize(vpKFs.size());

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];
      Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
      geometry_msgs::msg::Pose pose_msg;
      Eigen::Quaternionf q(Twc.topLeftCorner<3, 3>());
      q.normalize();
      pose_msg.position.x = Twc(0, 3);
      pose_msg.position.y = Twc(1, 3);
      pose_msg.position.z = Twc(2, 3);
      pose_msg.orientation.x = q.x();
      pose_msg.orientation.y = q.y();
      pose_msg.orientation.z = q.z();
      pose_msg.orientation.w = q.w();
      keyframe_msg.poses[i] = pose_msg;
    }

    mKeyFramePublisher->publish(keyframe_msg);

    visualization_msgs::msg::Marker covizGraph;
    covizGraph.header.frame_id = "world";
    covizGraph.header.stamp = timestamp;
    covizGraph.ns = "coviz_graph";
    covizGraph.id = 0;
    covizGraph.type = visualization_msgs::msg::Marker::LINE_LIST;
    covizGraph.action = visualization_msgs::msg::Marker::ADD;
    covizGraph.pose.orientation.w = 1.0;
    covizGraph.scale.x = 0.01;
    covizGraph.color.r = 1.0;
    covizGraph.color.g = 1.0;
    covizGraph.color.b = 1.0;
    covizGraph.color.a = 1.0;

    // publish line list
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      // Covisibility Graph
      const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
      Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
      geometry_msgs::msg::Point p1;
      p1.x = Ow.x();
      p1.y = Ow.y();
      p1.z = Ow.z();

      if (!vCovKFs.empty())
      {
        for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
        {
          if ((*vit)->mnId < vpKFs[i]->mnId)
            continue;
          Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
          geometry_msgs::msg::Point p2;
          p2.x = Ow2.x();
          p2.y = Ow2.y();
          p2.z = Ow2.z();
          covizGraph.points.push_back(p1);
          covizGraph.points.push_back(p2);
        }
      }

      KeyFrame *pParent = vpKFs[i]->GetParent();
      if (pParent)
      {
        Eigen::Vector3f Owp = pParent->GetCameraCenter();
        geometry_msgs::msg::Point p2;
        p2.x = Owp.x();
        p2.y = Owp.y();
        p2.z = Owp.z();
        covizGraph.points.push_back(p1);
        covizGraph.points.push_back(p2);
      }

      // Loops
      set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
      for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++)
      {
        if ((*sit)->mnId < vpKFs[i]->mnId)
          continue;
        Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
        geometry_msgs::msg::Point p2;
        p2.x = Owl.x();
        p2.y = Owl.y();
        p2.z = Owl.z();
        covizGraph.points.push_back(p1);
        covizGraph.points.push_back(p2);
      }
    }

    mCovizGraphPublisher->publish(covizGraph);
  }

  Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
  {
    if (mSensor != STEREO && mSensor != IMU_STEREO)
    {
      cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
      exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify())
    {
      cv::Mat M1l = settings_->M1l();
      cv::Mat M2l = settings_->M2l();
      cv::Mat M1r = settings_->M1r();
      cv::Mat M2r = settings_->M2r();

      cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
      cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (settings_ && settings_->needToResize())
    {
      cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
      cv::resize(imRight, imRightToFeed, settings_->newImSize());
    }
    else
    {
      imLeftToFeed = imLeft.clone();
      imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
        mbResetActiveMap = false;
      }
      else if (mbResetActiveMap)
      {
        mpTracker->ResetActiveMap();
        mbResetActiveMap = false;
      }
    }

    if (mSensor == System::IMU_STEREO)
      for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);

    // std::cout << "out grabber" << std::endl;

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
  }

  Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
  {
    if (mSensor != RGBD && mSensor != IMU_RGBD)
    {
      cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
      exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if (settings_ && settings_->needToResize())
    {
      cv::Mat resizedIm;
      cv::resize(im, resizedIm, settings_->newImSize());
      imToFeed = resizedIm;

      cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
        mbResetActiveMap = false;
      }
      else if (mbResetActiveMap)
      {
        mpTracker->ResetActiveMap();
        mbResetActiveMap = false;
      }
    }

    if (mSensor == System::IMU_RGBD)
      for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
  }

  Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point> &vImuMeas, string filename)
  {

    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbShutDown)
        return Sophus::SE3f();
    }

    if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
    {
      cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
      exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize())
    {
      cv::Mat resizedIm;
      cv::resize(im, resizedIm, settings_->newImSize());
      imToFeed = resizedIm;
    }

    // Check mode change
    {
      unique_lock<mutex> lock(mMutexMode);
      if (mbActivateLocalizationMode)
      {
        mpLocalMapper->RequestStop();

        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
          usleep(1000);
        }

        mpTracker->InformOnlyTracking(true);
        mbActivateLocalizationMode = false;
      }
      if (mbDeactivateLocalizationMode)
      {
        mpTracker->InformOnlyTracking(false);
        mpLocalMapper->Release();
        mbDeactivateLocalizationMode = false;
      }
    }

    // Check reset
    {
      unique_lock<mutex> lock(mMutexReset);
      if (mbReset)
      {
        mpTracker->Reset();
        mbReset = false;
        mbResetActiveMap = false;
      }
      else if (mbResetActiveMap)
      {
        cout << "SYSTEM-> Reseting active map in monocular case" << endl;
        mpTracker->ResetActiveMap();
        mbResetActiveMap = false;
      }
    }

    if (mSensor == System::IMU_MONOCULAR)
      for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
  }

  void System::ActivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
  }

  void System::DeactivateLocalizationMode()
  {
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
  }

  bool System::MapChanged()
  {
    static int n = 0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if (n < curn)
    {
      n = curn;
      return true;
    }
    else
      return false;
  }

  void System::Reset()
  {
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
  }

  void System::ResetActiveMap()
  {
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
  }

  void System::Shutdown()
  {
    {
      unique_lock<mutex> lock(mMutexReset);
      mbShutDown = true;
    }

    cout << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    /*if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }*/

    // Wait until all thread have effectively stopped
    /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;*/
    /*if(!mpLoopCloser->isFinished())
        cout << "mpLoopCloser is not finished" << endl;
    if(mpLoopCloser->isRunningGBA()){
        cout << "mpLoopCloser is running GBA" << endl;
        cout << "break anyway..." << endl;
        break;
    }*/
    /*usleep(5000);
}*/

    if (!mStrSaveAtlasToFile.empty())
    {
      Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
      SaveAtlas(FileType::BINARY_FILE);
    }

    /*if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif
  }

  bool System::isShutDown()
  {
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
  }

  void System::SaveTrajectoryTUM(const string &filename)
  {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
      cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
      return;
    }

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                      lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++)
    {
      if (*lbL)
        continue;

      KeyFrame *pKF = *lRit;

      Sophus::SE3f Trw;

      // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
      while (pKF->isBad())
      {
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose() * Two;

      Sophus::SE3f Tcw = (*lit) * Trw;
      Sophus::SE3f Twc = Tcw.inverse();

      Eigen::Vector3f twc = Twc.translation();
      Eigen::Quaternionf q = Twc.unit_quaternion();

      f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
  }

  void System::SaveKeyFrameTrajectoryTUM(const string &filename)
  {
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];

      // pKF->SetPose(pKF->GetPose()*Two);

      if (pKF->isBad())
        continue;

      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
        << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }

    f.close();
  }

  void System::SaveTrajectoryEuRoC(const string &filename)
  {

    cout << endl
         << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map *> vpMaps = mpAtlas->GetAllMaps();
    int numMaxKFs = 0;
    Map *pBiggerMap;
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for (Map *pMap : vpMaps)
    {
      std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
      if (pMap->GetAllKeyFrames().size() > numMaxKFs)
      {
        numMaxKFs = pMap->GetAllKeyFrames().size();
        pBiggerMap = pMap;
      }
    }

    vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      Twb = vpKFs[0]->GetImuPose();
    else
      Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

    for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++)
    {
      // cout << "1" << endl;
      if (*lbL)
        continue;

      KeyFrame *pKF = *lRit;
      // cout << "KF: " << pKF->mnId << endl;

      Sophus::SE3f Trw;

      // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
      if (!pKF)
        continue;

      // cout << "2.5" << endl;

      while (pKF->isBad())
      {
        // cout << " 2.bad" << endl;
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
        // cout << "--Parent KF: " << pKF->mnId << endl;
      }

      if (!pKF || pKF->GetMap() != pBiggerMap)
      {
        // cout << "--Parent KF is from another map" << endl;
        continue;
      }

      // cout << "3" << endl;

      Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

      // cout << "4" << endl;

      if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      {
        Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
        Eigen::Quaternionf q = Twb.unit_quaternion();
        Eigen::Vector3f twb = Twb.translation();
        f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
      else
      {
        Sophus::SE3f Twc = ((*lit) * Trw).inverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f twc = Twc.translation();
        f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }

      // cout << "5" << endl;
    }
    // cout << "end saving trajectory" << endl;
    f.close();
    cout << endl
         << "End of saving trajectory to " << filename << " ..." << endl;
  }

  void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
  {

    cout << endl
         << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    int numMaxKFs = 0;

    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      Twb = vpKFs[0]->GetImuPose();
    else
      Twb = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

    for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
              lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++, lbL++)
    {
      // cout << "1" << endl;
      if (*lbL)
        continue;

      KeyFrame *pKF = *lRit;
      // cout << "KF: " << pKF->mnId << endl;

      Sophus::SE3f Trw;

      // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
      if (!pKF)
        continue;

      // cout << "2.5" << endl;

      while (pKF->isBad())
      {
        // cout << " 2.bad" << endl;
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
        // cout << "--Parent KF: " << pKF->mnId << endl;
      }

      if (!pKF || pKF->GetMap() != pMap)
      {
        // cout << "--Parent KF is from another map" << endl;
        continue;
      }

      // cout << "3" << endl;

      Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

      // cout << "4" << endl;

      if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      {
        Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
        Eigen::Quaternionf q = Twb.unit_quaternion();
        Eigen::Vector3f twb = Twb.translation();
        f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
      else
      {
        Sophus::SE3f Twc = ((*lit) * Trw).inverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f twc = Twc.translation();
        f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }

      // cout << "5" << endl;
    }
    // cout << "end saving trajectory" << endl;
    f.close();
    cout << endl
         << "End of saving trajectory to " << filename << " ..." << endl;
  }

  /*void System::SaveTrajectoryEuRoC(const string &filename)
  {

      cout << endl << "Saving trajectory to " << filename << " ..." << endl;
      if(mSensor==MONOCULAR)
      {
          cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
          return;
      }

      vector<Map*> vpMaps = mpAtlas->GetAllMaps();
      Map* pBiggerMap;
      int numMaxKFs = 0;
      for(Map* pMap :vpMaps)
      {
          if(pMap->GetAllKeyFrames().size() > numMaxKFs)
          {
              numMaxKFs = pMap->GetAllKeyFrames().size();
              pBiggerMap = pMap;
          }
      }

      vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
      sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
      if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
          Twb = vpKFs[0]->GetImuPose_();
      else
          Twb = vpKFs[0]->GetPoseInverse_();

      ofstream f;
      f.open(filename.c_str());
      // cout << "file open" << endl;
      f << fixed;

      // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
      // We need to get first the keyframe pose and then concatenate the relative transformation.
      // Frames not localized (tracking failure) are not saved.

      // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
      // which is true when tracking failed (lbL).
      list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
      list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
      list<bool>::iterator lbL = mpTracker->mlbLost.begin();

      //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
      //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
      //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
      //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


      for(list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
          lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
      {
          //cout << "1" << endl;
          if(*lbL)
              continue;


          KeyFrame* pKF = *lRit;
          //cout << "KF: " << pKF->mnId << endl;

          Sophus::SE3f Trw;

          // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
          if (!pKF)
              continue;

          //cout << "2.5" << endl;

          while(pKF->isBad())
          {
              //cout << " 2.bad" << endl;
              Trw = Trw * pKF->mTcp;
              pKF = pKF->GetParent();
              //cout << "--Parent KF: " << pKF->mnId << endl;
          }

          if(!pKF || pKF->GetMap() != pBiggerMap)
          {
              //cout << "--Parent KF is from another map" << endl;
              continue;
          }

          //cout << "3" << endl;

          Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

          // cout << "4" << endl;


          if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
          {
              Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
              Sophus::SE3f Twb = Tbw.inverse();

              Eigen::Vector3f twb = Twb.translation();
              Eigen::Quaternionf q = Twb.unit_quaternion();
              f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
          }
          else
          {
              Sophus::SE3f Tcw = (*lit) * Trw;
              Sophus::SE3f Twc = Tcw.inverse();

              Eigen::Vector3f twc = Twc.translation();
              Eigen::Quaternionf q = Twc.unit_quaternion();
              f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
          }

          // cout << "5" << endl;
      }
      //cout << "end saving trajectory" << endl;
      f.close();
      cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
  }*/

  /*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
  {
      cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

      vector<Map*> vpMaps = mpAtlas->GetAllMaps();
      Map* pBiggerMap;
      int numMaxKFs = 0;
      for(Map* pMap :vpMaps)
      {
          if(pMap->GetAllKeyFrames().size() > numMaxKFs)
          {
              numMaxKFs = pMap->GetAllKeyFrames().size();
              pBiggerMap = pMap;
          }
      }

      vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
      sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      ofstream f;
      f.open(filename.c_str());
      f << fixed;

      for(size_t i=0; i<vpKFs.size(); i++)
      {
          KeyFrame* pKF = vpKFs[i];

         // pKF->SetPose(pKF->GetPose()*Two);

          if(pKF->isBad())
              continue;
          if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
          {
              cv::Mat R = pKF->GetImuRotation().t();
              vector<float> q = Converter::toQuaternion(R);
              cv::Mat twb = pKF->GetImuPosition();
              f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

          }
          else
          {
              cv::Mat R = pKF->GetRotation();
              vector<float> q = Converter::toQuaternion(R);
              cv::Mat t = pKF->GetCameraCenter();
              f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
          }
      }
      f.close();
  }*/

  void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
  {
    cout << endl
         << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map *> vpMaps = mpAtlas->GetAllMaps();
    Map *pBiggerMap;
    int numMaxKFs = 0;
    for (Map *pMap : vpMaps)
    {
      if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
      {
        numMaxKFs = pMap->GetAllKeyFrames().size();
        pBiggerMap = pMap;
      }
    }

    if (!pBiggerMap)
    {
      std::cout << "There is not a map!!" << std::endl;
      return;
    }

    vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];

      // pKF->SetPose(pKF->GetPose()*Two);

      if (!pKF || pKF->isBad())
        continue;
      if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      {
        Sophus::SE3f Twb = pKF->GetImuPose();
        Eigen::Quaternionf q = Twb.unit_quaternion();
        Eigen::Vector3f twb = Twb.translation();
        f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
      else
      {
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
    }
    f.close();
  }

  void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
  {
    cout << endl
         << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
      KeyFrame *pKF = vpKFs[i];

      if (!pKF || pKF->isBad())
        continue;
      if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor == IMU_RGBD)
      {
        Sophus::SE3f Twb = pKF->GetImuPose();
        Eigen::Quaternionf q = Twb.unit_quaternion();
        Eigen::Vector3f twb = Twb.translation();
        f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
      else
      {
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
      }
    }
    f.close();
  }

  /*void System::SaveTrajectoryKITTI(const string &filename)
  {
      cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
      if(mSensor==MONOCULAR)
      {
          cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
          return;
      }

      vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
      sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

      // Transform all keyframes so that the first keyframe is at the origin.
      // After a loop closure the first keyframe might not be at the origin.
      cv::Mat Two = vpKFs[0]->GetPoseInverse();

      ofstream f;
      f.open(filename.c_str());
      f << fixed;

      // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
      // We need to get first the keyframe pose and then concatenate the relative transformation.
      // Frames not localized (tracking failure) are not saved.

      // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
      // which is true when tracking failed (lbL).
      list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
      list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
      for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
      {
          ORB_SLAM3::KeyFrame* pKF = *lRit;

          cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

          while(pKF->isBad())
          {
              Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
              pKF = pKF->GetParent();
          }

          Trw = Trw * pKF->GetPoseCv() * Two;

          cv::Mat Tcw = (*lit)*Trw;
          cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
          cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

          f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
               Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
               Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
      }
      f.close();
  }*/

  void System::SaveTrajectoryKITTI(const string &filename)
  {
    cout << endl
         << "Saving camera trajectory to " << filename << " ..." << endl;
    if (mSensor == MONOCULAR)
    {
      cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
      return;
    }

    vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                      lend = mpTracker->mlRelativeFramePoses.end();
         lit != lend; lit++, lRit++, lT++)
    {
      ORB_SLAM3::KeyFrame *pKF = *lRit;

      Sophus::SE3f Trw;

      if (!pKF)
        continue;

      while (pKF->isBad())
      {
        Trw = Trw * pKF->mTcp;
        pKF = pKF->GetParent();
      }

      Trw = Trw * pKF->GetPose() * Tow;

      Sophus::SE3f Tcw = (*lit) * Trw;
      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Matrix3f Rwc = Twc.rotationMatrix();
      Eigen::Vector3f twc = Twc.translation();

      f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
    }
    f.close();
  }

  void System::SaveDebugData(const int &initIdx)
  {
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << "," << mpLocalMapper->mRwg(0, 2) << endl;
    f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << "," << mpLocalMapper->mRwg(1, 2) << endl;
    f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << "," << mpLocalMapper->mRwg(2, 2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt", ios_base::app);
    f << fixed;
    for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
    {
      for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
      {
        if (j != 0)
          f << ",";
        f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
      }
      f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
  }

  int System::GetTrackingState()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
  }

  vector<MapPoint *> System::GetTrackedMapPoints()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
  }

  vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
  {
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
  }

  double System::GetTimeFromIMUInit()
  {
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
      return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    else
      return 0.f;
  }

  bool System::isLost()
  {
    if (!mpAtlas->isImuInitialized())
      return false;
    else
    {
      if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
        return true;
      else
        return false;
    }
  }

  bool System::isFinished()
  {
    return (GetTimeFromIMUInit() > 0.1);
  }

  void System::ChangeDataset()
  {
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
      mpTracker->ResetActiveMap();
    }
    else
    {
      mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
  }

  float System::GetImageScale()
  {
    return mpTracker->GetImageScale();
  }

#ifdef REGISTER_TIMES
  void System::InsertRectTime(double &time)
  {
    mpTracker->vdRectStereo_ms.push_back(time);
  }

  void System::InsertResizeTime(double &time)
  {
    mpTracker->vdResizeImage_ms.push_back(time);
  }

  void System::InsertTrackTime(double &time)
  {
    mpTracker->vdTrackTotal_ms.push_back(time);
  }
#endif

  void System::SaveAtlas(int type)
  {
    if (!mStrSaveAtlasToFile.empty())
    {
      // clock_t start = clock();

      // Save the current session
      mpAtlas->PreSave();

      string pathSaveFileName = "./";
      pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
      pathSaveFileName = pathSaveFileName.append(".osa");

      string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
      std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
      string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

      if (type == TEXT_FILE) // File text
      {
        cout << "Starting to write the save text file " << endl;
        std::remove(pathSaveFileName.c_str());
        std::ofstream ofs(pathSaveFileName, std::ios::binary);
        boost::archive::text_oarchive oa(ofs);

        oa << strVocabularyName;
        oa << strVocabularyChecksum;
        oa << mpAtlas;
        cout << "End to write the save text file" << endl;
      }
      else if (type == BINARY_FILE) // File binary
      {
        cout << "Starting to write the save binary file" << endl;
        std::remove(pathSaveFileName.c_str());
        std::ofstream ofs(pathSaveFileName, std::ios::binary);
        boost::archive::binary_oarchive oa(ofs);
        oa << strVocabularyName;
        oa << strVocabularyChecksum;
        oa << mpAtlas;
        cout << "End to write save binary file" << endl;
      }
    }
  }

  bool System::LoadAtlas(int type)
  {
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    string pathLoadFileName = "./";
    pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
    pathLoadFileName = pathLoadFileName.append(".osa");

    if (type == TEXT_FILE) // File text
    {
      cout << "Starting to read the save text file " << endl;
      std::ifstream ifs(pathLoadFileName, std::ios::binary);
      if (!ifs.good())
      {
        cout << "Load file not found" << endl;
        return false;
      }
      boost::archive::text_iarchive ia(ifs);
      ia >> strFileVoc;
      ia >> strVocChecksum;
      ia >> mpAtlas;
      cout << "End to load the save text file " << endl;
      isRead = true;
    }
    else if (type == BINARY_FILE) // File binary
    {
      cout << "Starting to read the save binary file" << endl;
      std::ifstream ifs(pathLoadFileName, std::ios::binary);
      if (!ifs.good())
      {
        cout << "Load file not found" << endl;
        return false;
      }
      boost::archive::binary_iarchive ia(ifs);
      ia >> strFileVoc;
      ia >> strVocChecksum;
      ia >> mpAtlas;
      cout << "End to load the save binary file" << endl;
      isRead = true;
    }

    if (isRead)
    {
      // Check if the vocabulary is the same
      string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

      if (strInputVocabularyChecksum.compare(strVocChecksum) != 0)
      {
        cout << "The vocabulary load isn't the same which the load session was created " << endl;
        cout << "-Vocabulary name: " << strFileVoc << endl;
        return false; // Both are differents
      }

      mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
      mpAtlas->SetORBVocabulary(mpVocabulary);
      mpAtlas->PostLoad();

      return true;
    }
    return false;
  }

  string System::CalculateCheckSum(string filename, int type)
  {
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if (type == BINARY_FILE) // Binary file
      flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if (!f.is_open())
    {
      cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
      return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init(&md5Context);
    while (int count = f.readsome(buffer, sizeof(buffer)))
    {
      MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context);

    for (int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
      char aux[10];
      sprintf(aux, "%02x", c[i]);
      checksum = checksum + aux;
    }

    return checksum;
  }

} // namespace ORB_SLAM

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<ORB_SLAM3::System>("orb_slam3_node");
  rclcpp::spin(ros_node);
  ros_node->Shutdown();
  rclcpp::shutdown();
  return 0;
}