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

/** \file PosLvNode.h
    \brief This file defines the PosLvNode class which implements the Applanix
           POS LV node.
  */

#ifndef POSLV_NODE_H
#define POSLV_NODE_H

#include <string>
#include <memory>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

class VehicleNavigationSolution;
class VehicleNavigationPerformance;
class TimeTaggedDMIData;
class TCPConnectionClient;

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace poslv {

  /** The class PosLvNode implements the Applanix POSL LV node.
      \brief POS LV node
    */
  class PosLvNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    PosLvNode(const ros::NodeHandle& nh);
    /// Copy constructor
    PosLvNode(const PosLvNode& other) = delete;
    /// Copy assignment operator
    PosLvNode& operator = (const PosLvNode& other) = delete;
    /// Move constructor
    PosLvNode(PosLvNode&& other) = delete;
    /// Move assignment operator
    PosLvNode& operator = (PosLvNode&& other) = delete;
    /// Destructor
    virtual ~PosLvNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Publishes the vehicle navigation solution message
    void publishVehicleNavigationSolution(const ros::Time& timestamp,
      const VehicleNavigationSolution& vns);
    /// Publishes the vehicle navigation performance message
    void publishVehicleNavigationPerformance(const ros::Time& timestamp,
      const VehicleNavigationPerformance& vnp);
    /// Publishes the time-tagged DMI message
    void publishTimeTaggedDMIData(const ros::Time& timestamp,
      const TimeTaggedDMIData& dmi);
    /// Diagnose the TCP connection
    void diagnoseTCPConnection(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Diagnose the navigation status for primary GPS
    void diagnoseNavStatusPrimary(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Diagnose the navigation status for secondary GPS
    void diagnoseNavStatusSecondary(
      diagnostic_updater::DiagnosticStatusWrapper& status);
    /// Diagnose the alignment status
    void diagnoseAlignStatus(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /// Retrieves parameters
    void getParameters();
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Vehicle navigation solution publisher
    ros::Publisher _vehicleNavigationSolutionPublisher;
    /// Vehicle navigation performance publisher
    ros::Publisher _vehicleNavigationPerformancePublisher;
    /// Time-tagged DMI data publisher
    ros::Publisher _timeTaggedDMIDataPublisher;
    /// Frame ID
    std::string _frameId;
    /// IP string
    std::string _deviceIpStr;
    /// Port
    int _devicePort;
    /// TCP connection
    std::shared_ptr<TCPConnectionClient> _tcpConnection;
    /// Retry timeout for TCP
    double _retryTimeout;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Frequency diagnostic for vehicle navigation solution
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _vnsFreq;
    /// Vehicle navigation solution minimum frequency
    double _vnsMinFreq;
    /// Vehicle navigation solution maximum frequency
    double _vnsMaxFreq;
    /// Frequency diagnostic for vehicle navigation performance
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _vnpFreq;
    /// Vehicle navigation performance minimum frequency
    double _vnpMinFreq;
    /// Vehicle navigation performance maximum frequency
    double _vnpMaxFreq;
    /// Frequency diagnostic for time-tagged DMI data
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _dmiFreq;
    /// Time-tagged DMI data minimum frequency
    double _dmiMinFreq;
    /// Time-tagged DMI data maximum frequency
    double _dmiMaxFreq;
    /// Alignement status
    uint8_t _alignStatus;
    /// Navigation solution status for primary GPS
    int8_t _navStatus1;
    /// Navigation solution status for secondary GPS
    int8_t _navStatus2;
    /// Queue depth
    int _queueDepth;
    /// Vehicle navigation solution packet counter
    long _vnsPacketCounter;
    /// Vehicle navigation performance packet counter
    long _vnpPacketCounter;
    /// DMI packet counter
    long _dmiPacketCounter;
    /** @}
      */

  };

}

#endif // POSLV_NODE_H
