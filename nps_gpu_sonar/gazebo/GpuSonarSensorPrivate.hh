/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _GAZEBO_SENSORS_GPUSONARSENSOR_PRIVATE_HH_
#define _GAZEBO_SENSORS_GPUSONARSENSOR_PRIVATE_HH_

#include <mutex>
#include <sdf/sdf.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \internal
    /// \brief GPU Sonar sensor private data.
    class GpuSonarSensorPrivate
    {
      /// \brief Scan SDF elementz.
      public: sdf::ElementPtr scanElem;

      /// \brief Horizontal SDF element.
      public: sdf::ElementPtr horzElem;

      /// \brief Vertical SDF element.
      public: sdf::ElementPtr vertElem;

      /// \brief Range SDF element.
      public: sdf::ElementPtr rangeElem;

      /// \brief Camera SDF element.
      public: sdf::ElementPtr cameraElem;

      /// \brief Horizontal ray count.
      public: unsigned int horzRayCount;

      /// \brief Vertical ray count.
      public: unsigned int vertRayCount;

      /// \brief Horizontal range count.
      public: unsigned int horzRangeCount;

      /// \brief Vertical range count.
      public: unsigned int vertRangeCount;

      /// \brief Range count ratio.
      public: double rangeCountRatio;

      /// \brief The minimum range.
      public: double rangeMin;

      /// \brief The maximum range.
      public: double rangeMax;

      /// \brief GPU sonar rendering.
      public: rendering::GpuSonarPtr sonarCam;

      /// \brief Mutex to protect getting ranges.
      public: std::mutex mutex;

      /// \brief Sonar message to publish data.
      public: msgs::LaserScanStamped sonarMsg;

      /// \brief Parent entity of gpu ray sensor
      public: physics::EntityPtr parentEntity;

      /// \brief Publisher to publish ray sensor data
      public: transport::PublisherPtr scanPub;

      /// \brief True if the sensor was rendered.
      public: bool rendered;
    };
  }
}
#endif
