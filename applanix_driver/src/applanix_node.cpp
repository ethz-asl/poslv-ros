#include <ros/ros.h>
#include <libposlv/com/TCPConnectionClient.h>
#include <libposlv/sensor/POSLVComTCP.h>
#include <libposlv/types/Packet.h>
#include <libposlv/types/Group.h>
#include <applanix/VehicleNavigationSolutionMsg.h>
#include <applanix/VehicleNavigationPerformanceMsg.h>
#include <libposlv/types/VehicleNavigationSolution.h>
#include <libposlv/types/VehicleNavigationPerformance.h>
#include <libposlv/exceptions/IOException.h>
#include <libposlv/exceptions/TypeCastException.h>

namespace applanix
{
  class ApplanixNode
  {
  public:
    ApplanixNode(const ros::NodeHandle & nh);
    ~ApplanixNode();

    void spin();
  
  private:
    void publishVehicleNavigationSolution(const ros::Time & stamp, const VehicleNavigationSolution & vn);
    void publishVehicleNavigationPerformance(const ros::Time & stamp, const VehicleNavigationPerformance & vn);

    ros::NodeHandle mNodeHandle;
    ros::Publisher mVehicleNavigationSolutionPublisher;
    ros::Publisher mVehicleNavigationPerformancePublisher;
    std::string mIp;
    int mPort;
    std::string mFrameId;
  };



  ApplanixNode::ApplanixNode(const ros::NodeHandle & nh)
    : mNodeHandle(nh)
  {
    mNodeHandle.param<std::string>("ip_address", mIp, "129.132.39.171");
    mNodeHandle.param<int>("port", mPort, 5603);
    mNodeHandle.param<std::string>("frame_id", mFrameId, "vehicle_base_link");
 
    int queueDepth = 100;
    mVehicleNavigationSolutionPublisher = mNodeHandle.advertise<applanix::VehicleNavigationSolutionMsg>("vehicle_navigation_solution",queueDepth);
    mVehicleNavigationPerformancePublisher = mNodeHandle.advertise<applanix::VehicleNavigationPerformanceMsg>("vehicle_navigation_performance",queueDepth);
  }

  ApplanixNode::~ApplanixNode()
  {

  }

  void ApplanixNode::publishVehicleNavigationPerformance(const ros::Time & stamp, const ::VehicleNavigationPerformance & vn)
  {
    boost::shared_ptr<applanix::VehicleNavigationPerformanceMsg> navMsg(new applanix::VehicleNavigationPerformanceMsg);
    navMsg->header.stamp = stamp;
    navMsg->header.frame_id = mFrameId;

    /// Time 1
    navMsg->TimeDistance.Time1 = vn.mTimeDistance.mTime1;
    /// Time 2 
    navMsg->TimeDistance.Time2 = vn.mTimeDistance.mTime2;
    /// Distance tag
    navMsg->TimeDistance.DistanceTag = vn.mTimeDistance.mDistanceTag;
    /// Time type
    navMsg->TimeDistance.TimeType = vn.mTimeDistance.mTimeType;
    /// Distance type
    navMsg->TimeDistance.DistanceType = vn.mTimeDistance.mDistanceType;

    
    /// North position RMS error
    navMsg->NorthPositionRMSError = vn.mNorthPositionRMSError;
    /// East position RMS error
    navMsg->EastPositionRMSError = vn.mEastPositionRMSError;
    /// Down position RMS error
    navMsg->DownPositionRMSError = vn.mDownPositionRMSError;
    /// North velocity RMS error
    navMsg->NorthVelocityRMSError = vn.mNorthVelocityRMSError;
    /// East velocity RMS error
    navMsg->EastVelocityRMSError = vn.mEastVelocityRMSError;
    /// Down velocity RMS error
    navMsg->DownVelocityRMSError = vn.mDownVelocityRMSError;
    /// Roll RMS error
    navMsg->RollRMSError = vn.mRollRMSError;
    /// Pitch RMS error
    navMsg->PitchRMSError = vn.mPitchRMSError;
    /// Heading RMS error
    navMsg->HeadingRMSError = vn.mHeadingRMSError;
    /// Error ellipsoid semi major
    navMsg->ErrorEllipsoidSemiMajor = vn.mErrorEllipsoidSemiMajor;
    /// Error ellipsoid semi minor
    navMsg->ErrorEllipsoidSemiMinor = vn.mErrorEllipsoidSemiMinor;
    /// Error ellipsoid orientation
    navMsg->ErrorEllipsoidOrientation = vn.mErrorEllipsoidOrientation;

    mVehicleNavigationPerformancePublisher.publish(navMsg);

  }

  void ApplanixNode::publishVehicleNavigationSolution(const ros::Time & stamp, const ::VehicleNavigationSolution & vn)
  {
    boost::shared_ptr<applanix::VehicleNavigationSolutionMsg> navMsg(new applanix::VehicleNavigationSolutionMsg);
    navMsg->header.stamp = stamp;
    navMsg->header.frame_id = mFrameId;

    /// Time 1
    navMsg->TimeDistance.Time1 = vn.mTimeDistance.mTime1;
    /// Time 2 
    navMsg->TimeDistance.Time2 = vn.mTimeDistance.mTime2;
    /// Distance tag
    navMsg->TimeDistance.DistanceTag = vn.mTimeDistance.mDistanceTag;
    /// Time type
    navMsg->TimeDistance.TimeType = vn.mTimeDistance.mTimeType;
    /// Distance type
    navMsg->TimeDistance.DistanceType = vn.mTimeDistance.mDistanceType;


    
    navMsg->Latitude = vn.mLatitude;
    /// Longitude
    navMsg->Longitude = vn.mLongitude;
    /// Altitude
    navMsg->Altitude = vn.mAltitude;
    /// North velocity
    navMsg->NorthVelocity = vn.mNorthVelocity;
    /// East velocity
    navMsg->EastVelocity = vn.mEastVelocity;
    /// Down velocity
    navMsg->DownVelocity = vn.mDownVelocity;
    /// Roll
    navMsg->Roll = vn.mRoll;
    /// Pitch
    navMsg->Pitch = vn.mPitch;
    /// Heading
    navMsg->Heading = vn.mHeading;
    /// Wander angel
    navMsg->WanderAngle = vn.mWanderAngle;
    /// Track angle
    navMsg->TrackAngle = vn.mTrackAngle;
    /// Speed
    navMsg->Speed = vn.mSpeed;
    /// Angular rate longitude
    navMsg->AngularRateLong = vn.mAngularRateLong;
    /// Angular rate transverse
    navMsg->AngularRateTrans = vn.mAngularRateTrans;
    /// Angular rate down
    navMsg->AngularRateDown = vn.mAngularRateDown;
    /// Acceleration longitude
    navMsg->AccLong = vn.mAccLong;
    /// Acceleration transverse
    navMsg->AccTrans = vn.mAccTrans;
    /// Acceleration down
    navMsg->AccDown = vn.mAccDown;
    /// Alignment status
    navMsg->AlignementStatus = vn.mAlignementStatus;
  
    mVehicleNavigationSolutionPublisher.publish(navMsg);
    
  }


  void ApplanixNode::spin()
  {
    
    TCPConnectionClient connection(mIp, mPort);
    POSLVComTCP device(connection);
    while (ros::ok())
      {

        try {
          std::shared_ptr<Packet> packet = device.readPacket();
          ros::Time stamp = ros::Time::now();
          if (packet == NULL) 
            {
              ROS_DEBUG("Dropping message...");
            }
          else
            {
              const Group& group = packet->groupCast();
              if (group.instanceOf<VehicleNavigationSolution>())
                {
                  const VehicleNavigationSolution& vn =
                    group.typeCast<VehicleNavigationSolution>();
                  publishVehicleNavigationSolution(stamp, vn);
                }
              else if (group.instanceOf<VehicleNavigationPerformance>())
                {
                  const VehicleNavigationPerformance& vn =
                    group.typeCast<VehicleNavigationPerformance>();
                  publishVehicleNavigationPerformance(stamp, vn);
                }
            }
        } 
        catch(const IOException & e)
          {
            ROS_WARN_STREAM("IO Exception: " << e.what() << ". Attempting to continue");
            
          } 
        ros::spinOnce();
      }


  }


} // namespace applanix

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"applanix");
  ros::NodeHandle nh("~");
  int returnValue = 0;

  try 
    {
      applanix::ApplanixNode an(nh);
      an.spin();
    }
  catch(const std::exception & e)
    {
      ROS_ERROR_STREAM("Exception: " << e.what());
      returnValue = -1;
    }
  catch(...)
    {
      ROS_ERROR_STREAM("Unknown Exception");
      returnValue = -2;
    }
  
  return returnValue;
  
}
