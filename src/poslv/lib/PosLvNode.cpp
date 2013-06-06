/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "PosLvNode.h"

#include <bitset>

#include <diagnostic_updater/publisher.h>

#include <boost/shared_ptr.hpp>

#include <libposlv/types/VehicleNavigationSolution.h>
#include <libposlv/types/VehicleNavigationPerformance.h>
#include <libposlv/types/TimeTaggedDMIData.h>
#include <libposlv/types/PrimaryGPSStatus.h>
#include <libposlv/types/SecondaryGPSStatus.h>
#include <libposlv/types/GAMSSolutionStatus.h>
#include <libposlv/types/IINSolutionStatus.h>
#include <libposlv/types/GeneralStatusFDIR.h>
#include <libposlv/com/TCPConnectionClient.h>
#include <libposlv/sensor/POSLVComTCP.h>
#include <libposlv/exceptions/IOException.h>
#include <libposlv/exceptions/SystemException.h>
#include <libposlv/exceptions/TypeCreationException.h>
#include <libposlv/base/Timer.h>
#include <libposlv/types/Packet.h>
#include <libposlv/types/Group.h>

#include "poslv/VehicleNavigationSolutionMsg.h"
#include "poslv/VehicleNavigationPerformanceMsg.h"
#include "poslv/TimeTaggedDMIDataMsg.h"

namespace poslv {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  PosLvNode::PosLvNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh),
      _alignStatus(8),
      _navStatus1(-1),
      _navStatus2(-1),
      _vnsPacketCounter(0),
      _vnpPacketCounter(0),
      _dmiPacketCounter(0),
      _lastVnsTimestamp(0),
      _lastInterVnsTime(0),
      _lastVnpTimestamp(0),
      _lastInterVnpTime(0),
      _lastDmiTimestamp(0),
      _lastInterDmiTime(0),
      _gamsStatus(7),
      _iinStatus(8),
      _generalStatusA(0),
      _generalStatusB(0),
      _generalStatusC(0),
      _fdirLevel1Status(0),
      _fdirLevel2Status(0),
      _fdirLevel4Status(0),
      _fdirLevel5Status(0) {
    _gpsStatusMsgs[-1] = "Unknown";
    _gpsStatusMsgs[0] = "No data from receiver";
    _gpsStatusMsgs[1] = "Horizontal C/A mode";
    _gpsStatusMsgs[2] = "3-dimension C/A mode";
    _gpsStatusMsgs[3] = "Horizontal DGPS mode";
    _gpsStatusMsgs[4] = "3-dimension DGPS mode";
    _gpsStatusMsgs[5] = "Float RTK mode";
    _gpsStatusMsgs[6] = "Integer wide lane RTK mode";
    _gpsStatusMsgs[7] = "Integer narrow lane RTK mode";
    _gpsStatusMsgs[8] = "P-code";
    _alignStatusMsgs[0] = "Full navigation";
    _alignStatusMsgs[1] = "Fine alignment active";
    _alignStatusMsgs[2] = "GC CHI 2";
    _alignStatusMsgs[3] = "PC CHI 2";
    _alignStatusMsgs[4] = "GC CHI 1";
    _alignStatusMsgs[5] = "PC CHI 1";
    _alignStatusMsgs[6] = "Coarse leveling active";
    _alignStatusMsgs[7] = "Initial solution assigned";
    _alignStatusMsgs[8] = "No valid solution";
    _gamsStatusMsgs[0] = "Fixed integer";
    _gamsStatusMsgs[1] = "Fixed integer test install data";
    _gamsStatusMsgs[2] = "Degraded fixed integer";
    _gamsStatusMsgs[3] = "Floated ambiguity";
    _gamsStatusMsgs[4] = "Degraded floated ambiguity";
    _gamsStatusMsgs[5] = "Solution without install data";
    _gamsStatusMsgs[6] = "Solution from navigator attitude and install data";
    _gamsStatusMsgs[7] = "No solution";
    _iinStatusMsgs[1] = "Fixed narrow lane RTK";
    _iinStatusMsgs[2] = "Fixed wide lane RTK";
    _iinStatusMsgs[3] = "Float RTK";
    _iinStatusMsgs[4] = "Code DGPS";
    _iinStatusMsgs[5] = "RTCM DGPS";
    _iinStatusMsgs[6] = "Autonmous (C/A)";
    _iinStatusMsgs[7] = "GPS navigation solution";
    _iinStatusMsgs[8] = "No solution";
    getParameters();
    _vehicleNavigationSolutionPublisher =
      _nodeHandle.advertise<poslv::VehicleNavigationSolutionMsg>(
      "vehicle_navigation_solution", _queueDepth);
    _vehicleNavigationPerformancePublisher =
      _nodeHandle.advertise<poslv::VehicleNavigationPerformanceMsg>(
      "vehicle_navigation_performance", _queueDepth);
    _timeTaggedDMIDataPublisher =
      _nodeHandle.advertise<poslv::TimeTaggedDMIDataMsg>(
      "time_tagged_dmi_data", _queueDepth);
    _updater.setHardwareID("POS LV 220");
    _updater.add("TCP connection", this, &PosLvNode::diagnoseTCPConnection);
    _updater.add("System status", this, &PosLvNode::diagnoseSystemStatus);
    _vnsFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "vehicle_navigation_solution", _updater,
      diagnostic_updater::FrequencyStatusParam(&_vnsMinFreq, &_vnsMaxFreq,
      0.1, 10)));
    _vnpFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "vehicle_navigation_performance", _updater,
      diagnostic_updater::FrequencyStatusParam(&_vnpMinFreq, &_vnpMaxFreq,
      0.1, 10)));
    _dmiFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "time_tagged_dmi_data", _updater,
      diagnostic_updater::FrequencyStatusParam(&_dmiMinFreq, &_dmiMaxFreq,
      0.1, 10)));
    _updater.force_update();
  }

  PosLvNode::~PosLvNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void PosLvNode::publishVehicleNavigationSolution(const ros::Time& timestamp,
      const VehicleNavigationSolution& vns) {
    boost::shared_ptr<poslv::VehicleNavigationSolutionMsg> vnsMsg(
      new poslv::VehicleNavigationSolutionMsg);
    vnsMsg->header.stamp = timestamp;
    vnsMsg->header.frame_id = _frameId;
    vnsMsg->header.seq = _vnsPacketCounter++;
    vnsMsg->timeDistance.time1 = vns.mTimeDistance.mTime1;
    vnsMsg->timeDistance.time2 = vns.mTimeDistance.mTime2;
    vnsMsg->timeDistance.distanceTag = vns.mTimeDistance.mDistanceTag;
    vnsMsg->timeDistance.timeType = vns.mTimeDistance.mTimeType;
    vnsMsg->timeDistance.distanceType = vns.mTimeDistance.mDistanceType;
    vnsMsg->latitude = vns.mLatitude;
    vnsMsg->longitude = vns.mLongitude;
    vnsMsg->altitude = vns.mAltitude;
    vnsMsg->northVelocity = vns.mNorthVelocity;
    vnsMsg->eastVelocity = vns.mEastVelocity;
    vnsMsg->downVelocity = vns.mDownVelocity;
    vnsMsg->roll = vns.mRoll;
    vnsMsg->pitch = vns.mPitch;
    vnsMsg->heading = vns.mHeading;
    vnsMsg->wanderAngle = vns.mWanderAngle;
    vnsMsg->trackAngle = vns.mTrackAngle;
    vnsMsg->speed = vns.mSpeed;
    vnsMsg->angularRateLong = vns.mAngularRateLong;
    vnsMsg->angularRateTrans = vns.mAngularRateTrans;
    vnsMsg->angularRateDown = vns.mAngularRateDown;
    vnsMsg->accLong = vns.mAccLong;
    vnsMsg->accTrans = vns.mAccTrans;
    vnsMsg->accDown = vns.mAccDown;
    vnsMsg->alignementStatus = vns.mAlignementStatus;
    _vehicleNavigationSolutionPublisher.publish(vnsMsg);
    _vnsFreq->tick();
  }

  void PosLvNode::publishVehicleNavigationPerformance(
      const ros::Time& timestamp, const VehicleNavigationPerformance& vnp) {
    boost::shared_ptr<poslv::VehicleNavigationPerformanceMsg> vnpMsg(
      new poslv::VehicleNavigationPerformanceMsg);
    vnpMsg->header.stamp = timestamp;
    vnpMsg->header.frame_id = _frameId;
    vnpMsg->header.seq = _vnpPacketCounter++;
    vnpMsg->timeDistance.time1 = vnp.mTimeDistance.mTime1;
    vnpMsg->timeDistance.time2 = vnp.mTimeDistance.mTime2;
    vnpMsg->timeDistance.distanceTag = vnp.mTimeDistance.mDistanceTag;
    vnpMsg->timeDistance.timeType = vnp.mTimeDistance.mTimeType;
    vnpMsg->timeDistance.distanceType = vnp.mTimeDistance.mDistanceType;
    vnpMsg->northPositionRMSError = vnp.mNorthPositionRMSError;
    vnpMsg->eastPositionRMSError = vnp.mEastPositionRMSError;
    vnpMsg->downPositionRMSError = vnp.mDownPositionRMSError;
    vnpMsg->northVelocityRMSError = vnp.mNorthVelocityRMSError;
    vnpMsg->eastVelocityRMSError = vnp.mEastVelocityRMSError;
    vnpMsg->downVelocityRMSError = vnp.mDownVelocityRMSError;
    vnpMsg->rollRMSError = vnp.mRollRMSError;
    vnpMsg->pitchRMSError = vnp.mPitchRMSError;
    vnpMsg->headingRMSError = vnp.mHeadingRMSError;
    vnpMsg->errorEllipsoidSemiMajor = vnp.mErrorEllipsoidSemiMajor;
    vnpMsg->errorEllipsoidSemiMinor = vnp.mErrorEllipsoidSemiMinor;
    vnpMsg->errorEllipsoidOrientation = vnp.mErrorEllipsoidOrientation;
    _vehicleNavigationPerformancePublisher.publish(vnpMsg);
    _vnpFreq->tick();
  }

  void PosLvNode::publishTimeTaggedDMIData(
      const ros::Time& timestamp, const TimeTaggedDMIData& dmi) {
    boost::shared_ptr<poslv::TimeTaggedDMIDataMsg> dmiMsg(
      new poslv::TimeTaggedDMIDataMsg);
    dmiMsg->header.stamp = timestamp;
    dmiMsg->header.frame_id = _frameId;
    dmiMsg->header.seq = _dmiPacketCounter++;
    dmiMsg->timeDistance.time1 = dmi.mTimeDistance.mTime1;
    dmiMsg->timeDistance.time2 = dmi.mTimeDistance.mTime2;
    dmiMsg->timeDistance.distanceTag = dmi.mTimeDistance.mDistanceTag;
    dmiMsg->timeDistance.timeType = dmi.mTimeDistance.mTimeType;
    dmiMsg->timeDistance.distanceType = dmi.mTimeDistance.mDistanceType;
    dmiMsg->signedDistanceTraveled = dmi.mSignedDistanceTraveled;
    dmiMsg->unsignedDistanceTraveled = dmi.mUnsignedDistanceTraveled;
    dmiMsg->dmiScaleFactor = dmi.mDMIScaleFactor;
    dmiMsg->dataStatus = dmi.mDataStatus;
    dmiMsg->dmiType = dmi.mDMIType;
    dmiMsg->dmiDataRate = dmi.mDMIDataRate;
    _timeTaggedDMIDataPublisher.publish(dmiMsg);
    _dmiFreq->tick();
  }

  void PosLvNode::diagnoseTCPConnection(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_tcpConnection && _tcpConnection->isOpen()) {
      if (_lastInterVnsTime)
        status.add("Inter VNS packet time [s]", _lastInterVnsTime);
      if (_lastInterVnpTime)
        status.add("Inter VNP packet time [s]", _lastInterVnpTime);
      if (_lastInterDmiTime)
        status.add("Inter DMI packet time [s]", _lastInterDmiTime);
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "TCP connection opened on %s:%d.",
        _tcpConnection->getServerIP().c_str(),
        _tcpConnection->getPort());
    }
    else
     status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
      "TCP connection closed on %s:%d.", _deviceIpStr.c_str(), _devicePort);
  }

  void PosLvNode::diagnoseSystemStatus(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    status.add("Alignment status", _alignStatusMsgs[_alignStatus]);
    status.add("Primary GPS status", _gpsStatusMsgs[_navStatus1]);
    status.add("Secondary GPS status", _gpsStatusMsgs[_navStatus2]);
    status.add("GAMS solution status", _gamsStatusMsgs[_gamsStatus]);
    status.add("IIN processing status", _iinStatusMsgs[_iinStatus]);
    std::bitset<32> statusA(_generalStatusA);
    if (statusA.test(7))
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Full navigation solution");
    else
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
        "Incomplete navigation solution");
  }

  void PosLvNode::spin() {
    _tcpConnection.reset(new TCPConnectionClient(_deviceIpStr, _devicePort));
    POSLVComTCP device(*_tcpConnection);
    Timer timer;
    while (_nodeHandle.ok()) {
      try {
        std::shared_ptr<Packet> packet = device.readPacket();
        const ros::Time timestamp = ros::Time::now();
        if (packet->instanceOfGroup()) {
          const Group& group = packet->groupCast();
          if (group.instanceOf<VehicleNavigationSolution>()) {
            const VehicleNavigationSolution& vns =
              group.typeCast<VehicleNavigationSolution>();
            publishVehicleNavigationSolution(timestamp, vns);
            if (_lastVnsTimestamp)
              _lastInterVnsTime = vns.mTimeDistance.mTime2 - _lastVnsTimestamp;
            _lastVnsTimestamp = vns.mTimeDistance.mTime2;
            _alignStatus = vns.mAlignementStatus;
          }
          else if (group.instanceOf<VehicleNavigationPerformance>()) {
            const VehicleNavigationPerformance& vnp =
              group.typeCast<VehicleNavigationPerformance>();
            publishVehicleNavigationPerformance(timestamp, vnp);
            if (_lastVnpTimestamp)
              _lastInterVnpTime = vnp.mTimeDistance.mTime2 - _lastVnpTimestamp;
            _lastVnpTimestamp = vnp.mTimeDistance.mTime2;
          }
          else if (group.instanceOf<TimeTaggedDMIData>()) {
            const TimeTaggedDMIData& dmi = group.typeCast<TimeTaggedDMIData>();
            publishTimeTaggedDMIData(timestamp, dmi);
            if (_lastDmiTimestamp)
              _lastInterDmiTime = dmi.mTimeDistance.mTime2 - _lastDmiTimestamp;
            _lastDmiTimestamp = dmi.mTimeDistance.mTime2;
          }
          else if (group.instanceOf<PrimaryGPSStatus>()) {
            const PrimaryGPSStatus& gps = group.typeCast<PrimaryGPSStatus>();
            _navStatus1 = gps.mNavigationSolutionStatus;
          }
          else if (group.instanceOf<SecondaryGPSStatus>()) {
            const SecondaryGPSStatus& gps =
              group.typeCast<SecondaryGPSStatus>();
            _navStatus2 = gps.mNavigationSolutionStatus;
          }
          else if (group.instanceOf<GAMSSolutionStatus>()) {
            const GAMSSolutionStatus& gams =
              group.typeCast<GAMSSolutionStatus>();
            _gamsStatus = gams.mSolutionStatus;
          }
          else if (group.instanceOf<IINSolutionStatus>()) {
            const IINSolutionStatus& iin =
              group.typeCast<IINSolutionStatus>();
            _iinStatus = iin.mIINProcessingStatus;
          }
          else if (group.instanceOf<GeneralStatusFDIR>()) {
            const GeneralStatusFDIR& stat =
              group.typeCast<GeneralStatusFDIR>();
            _generalStatusA = stat.mGeneralStatusA;
            _generalStatusB = stat.mGeneralStatusB;
            _generalStatusC = stat.mGeneralStatusC;
            _fdirLevel1Status = stat.mFDIRLevel1Status;
            _fdirLevel2Status = stat.mFDIRLevel2Status;
            _fdirLevel4Status = stat.mFDIRLevel4Status;
            _fdirLevel5Status = stat.mFDIRLevel5Status;
          }
        }
      }
      catch (const IOException& e) {
        ROS_WARN_STREAM("IOException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (const SystemException& e) {
        ROS_WARN_STREAM("SystemException: " << e.what());
        ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
        timer.sleep(_retryTimeout);
      }
      catch (const TypeCreationException<unsigned short>& e) {
      }
      _updater.update();
      ros::spinOnce();
    }
  }

  void PosLvNode::getParameters() {
    _nodeHandle.param<std::string>("ros/frame_id", _frameId,
      "/poslv_link");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<std::string>("connection/device_ip", _deviceIpStr,
      "129.132.39.171");
    _nodeHandle.param<int>("connection/device_port", _devicePort, 5602);
    _nodeHandle.param<double>("connection/retry_timeout", _retryTimeout, 1);
    _nodeHandle.param<double>("diagnostics/vns_min_freq", _vnsMinFreq, 80);
    _nodeHandle.param<double>("diagnostics/vns_max_freq", _vnsMaxFreq, 120);
    _nodeHandle.param<double>("diagnostics/vnp_min_freq", _vnpMinFreq, 0.8);
    _nodeHandle.param<double>("diagnostics/vnp_max_freq", _vnpMaxFreq, 1.2);
    _nodeHandle.param<double>("diagnostics/dmi_min_freq", _dmiMinFreq, 80);
    _nodeHandle.param<double>("diagnostics/dmi_max_freq", _dmiMaxFreq, 120);
  }

}
