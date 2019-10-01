#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include <vicon_stream/client.h>

#include <tms_msg_db/msg/tmsdb_stamped.hpp>
#include <tms_msg_db/msg/tmsdb.hpp>
#include <tms_msg_ss/msg/vicon_data.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>
#include <string>
#include <chrono>

using std::string;
// using namespace ViconDataStreamSDK;
using namespace ViconDataStreamSDK::CPP;
using namespace boost;
using namespace std;
using namespace std::chrono_literals;

// export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/lib/x86_64-linux-gnu/"

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

class ViconStream : public rclcpp::Node
{

public:
  //ViconStream(idSensor, idPlace, stream_mode, host_name, frame_id, update_time, isDebug)
  ViconStream()
  : Node("vicon_stream")
  , count_(0)
  , idPlace(5002)
  , stream_mode("ClientPull")
  , frame_id("/world")
  , update_time(0.01)
  , host_name("192.168.4.151:801")

    {
      // auto node = std::make_shared<rclcpp::Node>("debug");
      // node->get_parameter_or("debug", isDebug, isDebug);
    //nh_priv.param("debug", isDebug, isDebug);
    // Publishers
      // auto db_pub = std::make_shared<ViconStream>("tms_db_data");
    //  dbpub = this->create_publisher<std_msgs::msg::String>("~/tms_db_data");
    //db_pub = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 1);

    // TimerEvent
      init_vicon();
  
      // auto node = std::make_shared<rclcpp::Node>("debug");

      // node->get_parameter_or("debug", isDebug, isDebug);
    //nh_priv.param("debug", isDebug, isDebug);
    // Publishers
      // auto db_pub = std::make_shared<ViconStream>("tms_db_data");
      posepub = this->create_publisher<tms_msg_ss::msg::ViconData>("/vicon/output");
      dbpub = this->create_publisher<tms_msg_db::msg::TmsdbStamped>("/tms_db_data");
      checkerpub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/checker_box");

    //db_pub = nh.advertise< tms_msg_db::TmsdbStamped >("tms_db_data", 1);


    // TimerEvent
      updatetimer = this->create_wall_timer(100ms, std::bind(&ViconStream::updateCallback, this));
    }
private:
  // Sensor ID
  int32_t idSensor;
  // Place ID
  int32_t idPlace;
  // Parameters:
  string stream_mode;
  string host_name;

  // string frame_id;

  double update_time;
  // bool isDebug;
  // Access client for ViconSDK
  // ViconDataStreamSDK::CPP::Client MyClient;
  // Output_IsConnected Output = MyClient.IsConnected();

ViconDataStreamSDK::CPP::Client MyClient;
// Output_IsConnected Output = MyClient.IsConnected()
// // Output.Connected == false
// MyClient.Connect( "localhost" );
// Output_IsConnected Output = MyClient.IsConnected()
// // Output.Connected == true
// // (assuming localhost is serving)

bool init_vicon()
  {
    std::cout << "Connecting to Vicon DataStream SDK at " << host_name << " ..." << std::endl;

    while (!MyClient.IsConnected().Connected)
    {
      MyClient.Connect(host_name);
      sleep(1);
    }
    if(MyClient.IsConnected().Connected){
      std::cout << "Connected" << std::endl;
    }
    Output_SetStreamMode result;

    if (stream_mode == "ClientPull")
    {
      result = MyClient.SetStreamMode(StreamMode::ClientPull);
    }
    else
    {
      return false;
    }

    if (result.Result != Result::Success)
    {
      std::cout << "Set stream mode call failed -- shutting down" <<std::endl;
    }

    MyClient.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);  // 'Z-up'

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    std::cout <<"Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) << " Y-"
                                       << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
                                       << Adapt(_Output_GetAxisMapping.ZAxis) <<std::endl;
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "." << _Output_GetVersion.Minor << "."
                                << _Output_GetVersion.Point << std::endl;
    return true;
  }

   void updateCallback()
   //void updateCallback(const tms_msg_db::msg::TmsdbStamped::SharedPtr& detect)
 //void updateCallback(const ros::TimerEvent& e)

  {
    //auto gettms(const std::string & message_type)
    //tms_msg_db::msg::TmsdbStamped::SharedPtr& detect
    //auto stamp = detect->header.stamp;
    //std::string frame_id = detect->header.frame_id;
    //std::string frame_id = detect->frame_id;

    auto posemsg = tms_msg_ss::msg::ViconData();
    auto tmsdbstamped = tms_msg_db::msg::TmsdbStamped();
    auto tmsdb = tms_msg_db::msg::Tmsdb();
    auto now = rclcpp::Clock().now();

    tmsdb.type = "robot";
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // posepub->publish(message);
    tmsdbstamped.header.frame_id = "frame_id";
 	  tmsdbstamped.header.stamp = rclcpp::Clock().now();
    // tmsdb.tmsdb.header.stamp = now;
    tmsdbstamped.tmsdb.push_back(tmsdb);

    dbpub->publish(tmsdbstamped);

    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();

    std::cout << "Get Frame" << std::endl;
    while (MyClient.GetFrame().Result != Result::Success)
    {
      std::cout << "." << std::endl;
    }
  

    Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
    bool isDebug = true;
    if (isDebug)
    {
      std::cout << "Segment Data Enabled: " << Adapt(MyClient.IsSegmentDataEnabled().Enabled) << std::endl;
      std::cout << "Marker Data Enabled: " << Adapt(MyClient.IsMarkerDataEnabled().Enabled) << std::endl;
      std::cout << "Unlabeled Marker Data Enabled: " << Adapt(MyClient.IsUnlabeledMarkerDataEnabled().Enabled)
                << std::endl;
      std::cout << "Device Data Enabled: " << Adapt(MyClient.IsDeviceDataEnabled().Enabled) << std::endl;
    }

    if (isDebug)
      std::cout << "Waiting for new frame...";
    while (MyClient.GetFrame().Result != Result::Success)
    {
      std::cout << "." << std::endl;
    }
    unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
    if (isDebug)
      std::cout << "Subjects (" << SubjectCount << "):" << std::endl;
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex)
    {
      if (isDebug)
        std::cout << "  Subject #" << SubjectIndex << std::endl;

      // Get the subject name
      std::string SubjectName = MyClient.GetSubjectName(SubjectIndex).SubjectName;
      if (isDebug)
        std::cout << "    Name: " << SubjectName << std::endl;

      // Get the root segment
      std::string RootSegment = MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
      if (isDebug)
        std::cout << "    Root Segment: " << RootSegment << std::endl;

      // Count the number of segments
      unsigned int SegmentCount = MyClient.GetSegmentCount(SubjectName).SegmentCount;
      if (isDebug)
        std::cout << "    Segments (" << SegmentCount << "):" << std::endl;

      // tsuika shimasu
      unsigned int markerCount = MyClient.GetMarkerCount(SubjectName).MarkerCount;
      for(unsigned int markerIndex = 0; markerIndex < markerCount; ++markerIndex){
        string markerName = MyClient.GetMarkerName(SubjectName, markerIndex).MarkerName;
        std::cout << "      Marker #" << markerIndex << " "<< markerName << endl;

        Output_GetMarkerGlobalTranslation _Output_GetSegmentGlobalTranslation =
            MyClient.GetMarkerGlobalTranslation(SubjectName, markerName);
        if (isDebug)
          std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[1] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[2] << ") "
                    << Adapt(_Output_GetSegmentGlobalTranslation.Occluded) << std::endl;
      }


      for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount; ++SegmentIndex)
      {
        if (isDebug)
          std::cout << "      Segment #" << SegmentIndex << std::endl;

        // Get the segment name
        std::string SegmentName = MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
        if (isDebug)
          std::cout << "        Name: " << SegmentName << std::endl;
// Get the global segment translation
        Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
            MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
        
        if(_Output_GetSegmentGlobalTranslation.Result==ViconDataStreamSDK::CPP::Result::Success){
          printf("success");
        }else if(_Output_GetSegmentGlobalTranslation.Result==ViconDataStreamSDK::CPP::Result::NotConnected){
          printf("not connected");
        }else if(_Output_GetSegmentGlobalTranslation.Result==ViconDataStreamSDK::CPP::Result::NoFrame){
          printf("No Frame");
        }else if(_Output_GetSegmentGlobalTranslation.Result==ViconDataStreamSDK::CPP::Result::InvalidSubjectName){
          printf("InvalidSubjectName");
        }else if(_Output_GetSegmentGlobalTranslation.Result==ViconDataStreamSDK::CPP::Result::InvalidSegmentName){
          printf("InvalidSegmentName");
        }

        if(_Output_GetSegmentGlobalTranslation.Occluded){
          printf("occluded enable");
        }else{
          printf("occluded disable");
        }
        if (isDebug)
          std::cout << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[0] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[1] << ", "
                    << _Output_GetSegmentGlobalTranslation.Translation[2] << ") "
                    << Adapt(_Output_GetSegmentGlobalTranslation.Occluded) << std::endl;

        // Get the global segment rotation in quaternion co-ordinates
        Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
            MyClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);
        if (isDebug)
          std::cout << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]
                    << ", " << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1] << ", "
                    << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2] << ", "
                    << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] << ") "
                    << Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded) << std::endl;

        // Get the global segment rotation in EulerXYZ co-ordinates
        Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ =
            MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName);
        if (isDebug)
          std::cout << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
                    << ", " << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] << ", "
                    << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] << ") "
                    << Adapt(_Output_GetSegmentGlobalRotationEulerXYZ.Occluded) << std::endl;
       
    posemsg.header.frame_id = frame_id;
    posemsg.header.stamp = rclcpp::Clock().now();
    // posemsg.measuredtime = rclcpp::Clock().now();
    // posemsg.measuredtime = now;
    posemsg.subjectname = SubjectName;
    posemsg.segmentname = SegmentName;
    posemsg.translation.x = _Output_GetSegmentGlobalTranslation.Translation[0];
    posemsg.translation.y = _Output_GetSegmentGlobalTranslation.Translation[1];
    posemsg.translation.z = _Output_GetSegmentGlobalTranslation.Translation[2];
    posemsg.rotation.x = _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
    posemsg.rotation.y = _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
    posemsg.rotation.z = _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
    posemsg.rotation.w = _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
    posemsg.eulerxyz[0] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0];
    posemsg.eulerxyz[1] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1];
    posemsg.eulerxyz[2] = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2];
    posepub->publish(posemsg);
    
    if(SubjectName == "checker_box"){
         
            auto markerarray = visualization_msgs::msg::MarkerArray();
            auto marker = visualization_msgs::msg::Marker();

            marker.text = "Hello";
            marker.header.frame_id = "world_link";
            marker.header.stamp = rclcpp::Clock().now();
            marker.type = marker.SPHERE;
            marker.pose.position.x = posemsg.translation.x * 0.001;
            marker.pose.position.y = posemsg.translation.y * 0.001;
            marker.pose.position.z = posemsg.translation.z * 0.001;
            marker.pose.orientation.x = posemsg.rotation.x;
            marker.pose.orientation.y = posemsg.rotation.y;
            marker.pose.orientation.z = posemsg.rotation.z;
            marker.pose.orientation.w = posemsg.rotation.w;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            
            markerarray.markers.push_back(marker);
            checkerpub->publish(markerarray);
          
        }

    // unsigned int segments = sdk->GetSegmentCount(name).SegmentCount;
		// 	for (unsigned int j = 0 ; j < segments ; j++) {
		// 		std::string segment = sdk->GetSegmentName(name, j).SegmentName;
		// 		std::cout << "Writing global translation " << segment
		// 				<< " for " << name << std::endl;}

    // Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation(SubjectName, SegmentName);
    
    // Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = MyClient.GetSegmentGlobalRotationQuaternion(SubjectName, SegmentName);

    // Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName, SegmentName);
// publish to tms_db_writer
         int32_t id = 0;
        

        std::string name;
        std::vector< std::string > v_name;
        v_name.clear();
        boost::split(v_name, SubjectName, boost::is_any_of("#"));

        if (v_name.size() == 2)
        {
          std::stringstream ss;
          ss << v_name.at(0);
          ss >> id;
          name = v_name.at(1).c_str();
        }

        if (id == 2009)
        {
          
          rclcpp::Clock ros_clock;
          auto start = ros_clock.now();

          // tmsdb.time = ros_clock.now();
          tmsdb.id = id;
          tmsdb.name = name;
          tmsdb.x = posemsg.translation.x / 1000;
          tmsdb.y = posemsg.translation.y / 1000;
          tmsdb.z = posemsg.translation.z / 1000;
          // Vicon DataStream SDK: Rotations are expressed in radians.
          tmsdb.rr = posemsg.eulerxyz[0];
          tmsdb.rp = posemsg.eulerxyz[1];
          tmsdb.ry = posemsg.eulerxyz[2];
          tmsdb.place = idPlace;
          tmsdb.sensor = idSensor;
          tmsdb.state = 1;

          std::stringstream ss;
          ss.clear();
          ss << posemsg.eulerxyz[2];
          tmsdb.joint = ss.str();

          tmsdbstamped.tmsdb.push_back(tmsdb);
        }
        else if (id != 0 && posemsg.translation.x != 0 && posemsg.translation.y != 0)
        {
          
          // auto now = std::chrono::system_clock::now();
          // auto in_time_t = std::chrono::system_clock::to_time_t(now);
          // boost::posix_time::ptime t = boost::posix_time::second_clock::local_time(); 
          // auto ts = std::ctime(&in_time_t);
          // boost::posix_time::to_iso_extended_string(t);
          // tmsdb.time = boost::posix_time::to_iso_extended_string(t);

          auto now = std::chrono::system_clock::now();
          auto itt = std::chrono::system_clock::to_time_t(now);
          std::ostringstream ss;
          ss << std::put_time(gmtime(&itt), "%FT%TZ");
          // tmsdb.time = std::to_string(std::chrono::system_clock::now());
          tmsdb.time = ss.str();
          tmsdb.id = id;
          tmsdb.name = name;
          // Vicon DataStream SDK: Positions are expressed in millimeters.
          tmsdb.x = posemsg.translation.x / 1000;
          tmsdb.y = posemsg.translation.y / 1000;
          tmsdb.z = posemsg.translation.z / 1000;
          // Vicon DataStream SDK: Rotations are expressed in radians.
          tmsdb.rr = posemsg.eulerxyz[0];
          tmsdb.rp = posemsg.eulerxyz[1];
          tmsdb.ry = posemsg.eulerxyz[2];
          tmsdb.place = idPlace;
          tmsdb.sensor = idSensor;
          tmsdb.state = 1;

          tmsdbstamped.tmsdb.push_back(tmsdb);
        }
    //   }
    
    

    dbpub->publish(tmsdbstamped);
  

      }
    }
    
  }


  // Publisher
  rclcpp::Publisher<tms_msg_db::msg::TmsdbStamped>::SharedPtr dbpub;
  rclcpp::Publisher<tms_msg_ss::msg::ViconData>::SharedPtr posepub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr checkerpub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr markerpub;
  // Timer
  rclcpp::TimerBase::SharedPtr updatetimer;
  // Parameters:
  string frame_id;
  size_t count_;  
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  // ViconStream vs;
  rclcpp::spin(std::make_shared<ViconStream>());
  rclcpp::shutdown();

  return 0;
}
