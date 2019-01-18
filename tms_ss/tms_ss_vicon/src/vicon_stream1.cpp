//------------------------------------------------------------------------------
// @file   : vicon_stream.cpp
// @brief  : data stream using ViconSDK v1.3 / simple version
// @author : Minsoo Song
// @version: Ver1.0.0 (since 2014.05.02)
// @date   : 2018.12.20
//------------------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <vicon_stream/client.h>

#include <tms_msg_db/msg/tmsdb_stamped.h>
#include <tms_msg_db/msg/tmsdb.h>
#include <tms_msg_ss/msg/vicon_data.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <visualization_msgs/msg/marker.h>

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>
#include <string>
#include <chrono>





// #define rad2deg(x) ((x)*(180.0)/M_PI)
// #define deg2rad(x)  ((x)*M_PI/180.0)
//------------------------------------------------------------------------------
using std::string;
using namespace ViconDataStreamSDK::CPP;
using namespace boost;
using namespace std;
using namespace std::chrono_literals;

//------------------------------------------------------------------------------
string Adapt(const Direction::Enum i_Direction)
{
  switch (i_Direction)
  {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

//------------------------------------------------------------------------------
std::string Adapt(const bool i_Value)
{
  return i_Value ? "True" : "False";
}

//------------------------------------------------------------------------------
class ViconStream : public rclcpp::Node
{
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
public:
  //ViconStream(idSensor, idPlace, stream_mode, host_name, frame_id, update_time, isDebug)
  ViconStream()
  : Node("Vicon"), idSensor(3001), idPlace(5001), stream_mode("ClientPull"), host_name("192.168.4.151:801"), frame_id("/world"), update_time(0.01), isDebug(false)
    //rclcpp::Node::Node("Vicon")
   //nh_priv("~")
    // idSensor(3001)
    // ,  // vicon sensor
    // idPlace(5001)
    // ,  // 988 room
    // stream_mode("ClientPull")
    // , host_name("192.168.4.151:801")
    // , frame_id("/world")
    // , update_time(0.01)
    // ,  // sec
    // isDebug(false)
    {

      // idSensor = 3001;
      // idPlace = 5001;
      // stream_mode = "ClientPull";
      // host_name = "192.168.4.151:801";
      // frame_id = "/world";
      // update_time = 0.01;
      // isDebug = false;


    //isDebug(false)
  
    // Init parameter
      parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    //auto node = std::make_shared<rclcpp::Node>("stream_mode");
    //node->create_publisher<ViconStream>("~/stream_mode", stream_mode, stream_mode);
    //nh_priv.param("stream_mode", stream_mode, stream_mode);


    //auto node = std::make_shared<rclcpp::Node>("host_name");
    //node->create_publisher<ViconStream>("~/host_name", host_name,);
    //nh_priv.param("host_name", host_name, host_name);


    //auto node = std::make_shared<rclcpp::Node>("frame_id");
    //node->create_publisher<ViconStream>("~/frame_id", frame_id, frame_id);
    //nh_priv.param("frame_id", frame_id, frame_id);


    //auto node = std::make_shared<rclcpp::Node>("update_time_sec");
    //node->create_publisher<ViconStream>("~/update_time_sec", update_time, update_time);
    //nh_priv.param("update_time_sec", update_time, update_time);

    // Parameter ip_port_param;
    // this->get_parameter_or("ip_port",ip_port_param, Parameter("",80));
    // ip_port_ = ip_port_param.as_int();

      // auto node = std::make_shared<rclcpp::Node>("debug");
      // node->get_parameter_or("debug", isDebug, isDebug);
    //nh_priv.param("debug", isDebug, isDebug);


    // Init Vicon Stream
      init_vicon();
    // Publishers
    
      dbpub = this->create_publisher<std_msgs::msg::String>("~/tms_db_data");

      // auto db_pub = std::make_shared<rclcpp::Node>("tms_db_data");
      // db_pub->create_publisher<ViconStream>("~/tms_db_data", 1);
    //db_pub = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 1);

    　posepub = this->create_publisher<std_msgs::msg::String>("~/output");

      // auto pose_pub = std::make_shared<rclcpp::Node>("output");
      // pose_pub->create_publisher<ViconStream>("~/output", 1);
    //pose_pub = nh_priv.advertise< tms_msg_ss::vicon_data >("output", 1);

    　markerpub = this->create_publisher<std_msgs::msg::String>("~/checker_box");

      // auto box_pub = std::make_shared<rclcpp::Node>("checker_box");
      // box_pub->create_publisher<ViconStream>("~/checker_box", 1);
    //box_pub = nh.advertise< visualization_msgs::Marker>("checker_box",1);
    // TimerEvent
    update_timer = this->create_wall_timer(100ms, std::bind(&ViconStream::updateCallback), this);

    }
    //update_timer = nh.createTimer(ros::Duration(update_time), &ViconStream::updateCallback, this);
  

  //----------------------------------------------------------------------------
  ~ViconStream()
  {
    rclcpp::shutdown();
    //ROS_ASSERT(shutdown_vicon());
  }

  //------------------------------------------------------------------------------
private:
  // Sensor ID
  int32_t idSensor;
  // Place ID
  int32_t idPlace;
  // NodeHandle 

  //rclcpp::NodeHandle nh;
  //rclcpp::NodeHandle nh_priv;
  

  // Access client for ViconSDK
  ViconDataStreamSDK::CPP::Client MyClient;
  // Output_IsConnected Output = MyClient.IsConnected();

  bool init_vicon()
  //{
  {
    RCLCPP_INFO(this->get_logger(), "Connecting to Vicon DataStream SDK at " << host_name << " ...");

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(host_name);
      sleep(1);
      if (!ros::ok())
        return false;
    }
    // ROS_ASSERT(MyClient.IsConnected().Connected);
    RCLCPP_INFO(this->get_logger(), "... connected!");
    RCLCPP_INFO(this->get_logger(), "Setting Stream Mode to "
                    << "ClientPull");

    Output_SetStreamMode result;

    if (stream_mode == "ClientPull")
    {
      result = MyClient.SetStreamMode(StreamMode::ClientPull);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "stream_mode error");
      return false;
    }

    if (result.Result != Result::Success)
    {
      RCLCPP_FATAL(logger_,"Set stream mode call failed -- shutting down");
      rclcpp::shutdown();
    }

    MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);  // 'Z-up'

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    RCLCPP_INFO(this->get_logger(), "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
                                       << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
                                       << Adapt(_Output_GetAxisMapping.ZAxis));
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    RCLCPP_INFO(this->get_logger(), "Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
                                << _Output_GetVersion.Point);
    return true;
  }
    // while (!MyClient.IsConnected().Connected)
    // {
    //   MyClient.Connect(host_name);
    //   RCLCPP_INFO(this->get_logger(),".");
    //   sleep(1);
    //   rclcpp::executor::Executor::spin_once();
    //   if (!rclcpp::ok())
    //     return false;
    // }
    // //ROS_ASSERT(MyClient.IsConnected().Connected);
    // RCLCPP_INFO(this->get_logger(), "... connected!")
    // RCLCPP_INFO(this->get_logger(), "Setting Stream Mode to ClientPull");

    // Output_SetStreamMode result;

    // if (stream_mode == "ClientPull")
    // {
    //   result = MyClient.SetStreamMode(StreamMode::ClientPull);
    // }
    // else
    // {
    //   RCLCPP_INFO(this->get_logger(), "stream_mode error")
    //   return false;
    // }

    // if (result.Result != Result::Success)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Set stream mode call failed -- shutting down")
    //   rclcpp::shutdown();
    // }

    // MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);  // 'Z-up'

    // Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    // RCLCPP_INFO(this->get_logger(),"Axis Mapping: X-" + Adapt(_Output_GetAxisMapping.XAxis) + " Y-"
    //                                    + Adapt(_Output_GetAxisMapping.YAxis) + " Z-"
    //                                    + Adapt(_Output_GetAxisMapping.ZAxis));
    // Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    // RCLCPP_INFO(this->get_logger(),"Version: " + _Output_GetVersion.Major + "." + _Output_GetVersion.Minor + "."
    //                             + _Output_GetVersion.Point);
    // return true;
  
  //}

  //----------------------------------------------------------------------------
  // bool shutdown_vicon();
  // {
  //   auto node = rclcpp::Node::make_shared("api_composition");
  //   RCLCPP_INFO(node->get_logger(), "Disconnecting from Vicon DataStream SDK")
  //   MyClient.Disconnect();
  //   RCLCPP_INFO(node->get_logger(), "... disconnected.")
  //   return true;
  // }

  //----------------------------------------------------------------------------
  void updateCallback()
  //void updateCallback(const ros::TimerEvent& e)
  {
    auto tmsdb = tms_msg_db::TmsdbStamped();
    dbpub->publish(tmsdb);
    // tms_msg_db::TmsdbStamped tmsdb;
    // tmsdb.tmsdb.frame_id = frame_id;
    // tmsdb.tmsdb.stamp = now;

    // tmsdb.data = "Hello, world! " + std::to_string(count_++);    
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());


    auto posemsg = tms_msg_ss::ViconData();
    posepub->publish(posemsg);

    //     tms_msg_ss::ViconData pose_msg;
    //     now = rclcpp::Time::now() + rclcpp::Time(9 * 60 * 60); //ros::Duration(9 * 60 * 60);  // GMT +9

    //     pose_msg.header.frame_id = frame_id;
    //     pose_msg.header.stamp = now;
    //     pose_msg.measuredTime = now;
    //     pose_msg.subjectName = SubjectName;
    //     pose_msg.segmentName = SegmentName;
    //     pose_msg.translation.x = _Output_GetSegmentGlobalTranslation.Translation[0];
    //     pose_msg.translation.y = _Output_GetSegmentGlobalTranslation.Translation[1];
    //     pose_msg.translation.z = _Output_GetSegmentGlobalTranslation.Translation[2];
    //     pose_msg.rotation.x = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
    //     pose_msg.rotation.y = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
    //     pose_msg.rotation.z = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
    //     pose_msg.rotation.w = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
    //     pose_msg.eulerXYZ[0] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0];
    //     pose_msg.eulerXYZ[1] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1];
    //     pose_msg.eulerXYZ[2] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2];


    //     std::cout << _Output_GetFrameNumber.FrameNumber << "::" << SegmentName << std::endl;

    auto marker = visualization_msgs::Marker();
    markerpub->publish(marker);

    //     if(SubjectName == "checker_box"){
    //         visualization_msgs::Marker marker;
    //         marker.header.frame_id = "world_link";
    //         marker.header.stamp = rclcpp::Time::time();
    //         marker.type = visualization_msgs::Marker::SPHERE;
    //         marker.pose.position.x = pose_msg.translation.x * 0.001;
    //         marker.pose.position.y = pose_msg.translation.y * 0.001;
    //         marker.pose.position.z = pose_msg.translation.z * 0.001;
    //         marker.pose.orientation.x = pose_msg.rotation.x;
    //         marker.pose.orientation.y = pose_msg.rotation.y;
    //         marker.pose.orientation.z = pose_msg.rotation.z;
    //         marker.pose.orientation.w = pose_msg.rotation.w;
    //         marker.scale.x = 0.5;
    //         marker.scale.y = 0.5;
    //         marker.scale.z = 0.5;
    //         marker.color.a = 0.5;
    //         marker.color.r = 1.0;
    //         marker.color.g = 1.0;
    //         box_pub.publish(marker);
    //     }
    // Enable some different data types


    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();
    
    // bool isDebug()
    // if (isDebug)
    // {
    //   std::cout << "Segment Data Enabled: " << Adapt(MyClient.IsSegmentDataEnabled().Enabled) << std::endl;
    //   std::cout << "Marker Data Enabled: " << Adapt(MyClient.IsMarkerDataEnabled().Enabled) << std::endl;
    //   std::cout << "Unlabeled Marker Data Enabled: " << Adapt(MyClient.IsUnlabeledMarkerDataEnabled().Enabled)
    //             << std::endl;
    //   std::cout << "Device Data Enabled: " << Adapt(MyClient.IsDeviceDataEnabled().Enabled) << std::endl;
    // }

    // if (isDebug)
    //   std::cout << "Waiting for new frame...";
    // while (MyClient.GetFrame().Result != Result::Success)
    // {
    //   std::cout << "." << std::endl;
    // }

    // // Get the frame number
    // Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
    // if (isDebug)
    //   std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

    // // Count the number of subjects
    // unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
    // if (isDebug)
    //   std::cout << "Subjects (" << SubjectCount << "):" << std::endl;

    // rclcpp::Time now = rclcpp::Time::now() + rclcpp::Time(9 * 60 * 60); //ros::Duration(9 * 60 * 60); // GMT +9


    ///////////////////////////////////////////////////////////////////
    // for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    // {
    //   if (isDebug)
    //     std::cout << "  Subject #" << SubjectIndex << std::endl;

    //   // Get the subject name
    //   std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
    //   if (isDebug)
    //     std::cout << "    Name: " << SubjectName << std::endl;

    //   // Get the root segment
    //   std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
    //   if (isDebug)
    //     std::cout << "    Root Segment: " << RootSegment << std::endl;

    //   // Count the number of segments
    //   unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;
    //   if (isDebug)
    //     std::cout << "    Segments (" << SegmentCount << "):" << std::endl;

    //   for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
    //   {
    //     if (isDebug)
    //       std::cout << "      Segment #" << SegmentIndex << std::endl;

    //     // Get the segment name
    //     std::string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
    //     if (isDebug)
    //       std::cout << "        Name: " << SegmentName << std::endl;

    //     // Get the global segment translation
    //     Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
    //         MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
    //     if (isDebug)
    //       std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
    //                 << _Output_GetSegmentGlobalTranslation.Translation[1] << ", "
    //                 << _Output_GetSegmentGlobalTranslation.Translation[2] << ") "
    //                 << Adapt(_Output_GetSegmentGlobalTranslation.Occluded) << std::endl;

    //     // Get the global segment rotation in quaternion co-ordinates
    //     Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
    //         MyClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
    //     if (isDebug)
    //       std::cout << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]
    //                 << ", " << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1] << ", "
    //                 << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2] << ", "
    //                 << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] << ") "
    //                 << Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded) << std::endl;

    //     // Get the global segment rotation in EulerXYZ co-ordinates
    //     Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ =
    //         MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName);
    //     if (isDebug)
    //       std::cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
    //                 << ", " << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] << ", "
    //                 << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] << ") "
    //                 << Adapt(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded) << std::endl;


      //   //----------------------------------------------------------------------
      //   // publish to tms_db_writer
      //   int32_t id = 0;
      //   tms_msg_db::Tmsdb TmpData;

      //   std::string name;
      //   std::vector< std::string > v_name;
      //   v_name.clear();
      //   boost::split(v_name, SubjectName, boost::is_any_of("#"));

      //   if (v_name.size() == 2)
      //   {
      //     std::stringstream ss;
      //     ss << v_name.at(0);
      //     ss >> id;
      //     name = v_name.at(1).c_str();
      //   }

      //   if (id == 2009)
      //   {
      //     now = rclcpp::Clock();   // GMT +9

      //     TmpData.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      //     TmpData.id = id;
      //     TmpData.name = name;
      //     TmpData.x = pose_msg.translation.x / 1000;
      //     TmpData.y = pose_msg.translation.y / 1000;
      //     TmpData.z = pose_msg.translation.z / 1000;
      //     // Vicon DataStream SDK: Rotations are expressed in radians.
      //     TmpData.rr = pose_msg.eulerXYZ[0];
      //     TmpData.rp = pose_msg.eulerXYZ[1];
      //     TmpData.ry = pose_msg.eulerXYZ[2];
      //     TmpData.place = idPlace;
      //     TmpData.sensor = idSensor;
      //     TmpData.state = 1;

      //     std::stringstream ss;
      //     ss.clear();
      //     ss << pose_msg.eulerXYZ[2];
      //     TmpData.joint = ss.str();

      //     tmsdb.tmsdb.push_back(TmpData);
      //   }
      //   else if (id != 0 && pose_msg.translation.x != 0 && pose_msg.translation.y != 0)
      //   {
      //     now = rclcpp::Clock();    // GMT +9

      //     TmpData.time = boost::posix_time::to_iso_extended_string(now.toBoost());
      //     TmpData.id = id;
      //     TmpData.name = name;
      //     // Vicon DataStream SDK: Positions are expressed in millimeters.
      //     TmpData.x = pose_msg.translation.x / 1000;
      //     TmpData.y = pose_msg.translation.y / 1000;
      //     TmpData.z = pose_msg.translation.z / 1000;
      //     // Vicon DataStream SDK: Rotations are expressed in radians.
      //     TmpData.rr = pose_msg.eulerXYZ[0];
      //     TmpData.rp = pose_msg.eulerXYZ[1];
      //     TmpData.ry = pose_msg.eulerXYZ[2];
      //     TmpData.place = idPlace;
      //     TmpData.sensor = idSensor;
      //     TmpData.state = 1;

      //     tmsdb.tmsdb.push_back(TmpData);
      //   }
      // }

      // unsigned int MarkerCount = MyClient.GetMarkerCount( SubjectName ).MarkerCount;
      // std::cout << "    Markers (" << MarkerCount << "):" << std::endl;
      // for( unsigned int MarkerIndex = 0 ; MarkerIndex < MarkerCount ; ++MarkerIndex )
      // {
      //   // Get the marker name
      //   std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

      //   // Get the marker parent
      //   std::string MarkerParentName = MyClient.GetMarkerParentName( SubjectName, MarkerName ).SegmentName;

      //   // Get the global marker translation
      //   Output_GetMarkerGlobalTranslation _Output_GetMarkerGlobalTranslation =
      //     MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );

      //   std::cout << "      Marker #" << MarkerIndex            << ": "
      //                                 << MarkerName             << " ("
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 0 ]  << ", "
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 1 ]  << ", "
      //                                 << _Output_GetMarkerGlobalTranslation.Translation[ 2 ]  << ") "
      //                                 << std::endl;
      // }

  // Publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dbpub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posepub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr markerpub;
  // Timer
  rclcpp::TimerBase::SharedPtr update_timer;
  // Parameters:
  string stream_mode;
  string host_name;
  string frame_id;
  size_t count_;
  double update_time;
  bool isDebug;

    }
  
};

//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "vicon_stream");
  // ViconStream vs;
  rclcpp::spin(std::make_shared<ViconStream>());
  rclcpp::shutdown();

  return 0;
}

//------------------------------------------------------------------------------
// EOF
