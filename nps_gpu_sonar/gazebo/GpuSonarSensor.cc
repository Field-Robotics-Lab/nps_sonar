/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>
#include <functional>
#include <ignition/math.hh>
#include <ignition/math/Helpers.hh>
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderEngine.hh"
//#include "gazebo/rendering/GpuSonar.hh"
#include "../rendering/GpuSonar.hh"

#include "gazebo/sensors/Noise.hh"
#include "gazebo/sensors/SensorFactory.hh"
//#include "gazebo/sensors/GpuSonarSensorPrivate.hh"
#include "GpuSonarSensorPrivate.hh"
//#include "gazebo/sensors/GpuSonarSensor.hh"
#include "GpuSonarSensor.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("gpu_sonar", GpuSonarSensor)

//////////////////////////////////////////////////
GpuSonarSensor::GpuSonarSensor()
: Sensor(sensors::IMAGE),
  dataPtr(new GpuSonarSensorPrivate)
{
  this->dataPtr->rendered = false;
  this->active = false;
  this->connections.push_back(
      event::Events::ConnectRender(
        std::bind(&GpuSonarSensor::Render, this)));
}

//////////////////////////////////////////////////
GpuSonarSensor::~GpuSonarSensor()
{
  this->Fini();
}

//////////////////////////////////////////////////
std::string GpuSonarSensor::Topic() const
{
  std::string topicName = "~/";
  topicName += this->ParentName() + "/" + this->Name() + "/scan";
  boost::replace_all(topicName, "::", "/");

  return topicName;
}

//////////////////////////////////////////////////
void GpuSonarSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

//////////////////////////////////////////////////
void GpuSonarSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->dataPtr->scanPub =
    this->node->Advertise<msgs::LaserScanStamped>(this->Topic(), 50);

  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  this->dataPtr->scanElem = rayElem->GetElement("scan");
  this->dataPtr->horzElem = this->dataPtr->scanElem->GetElement("horizontal");
  this->dataPtr->rangeElem = rayElem->GetElement("range");

  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem = this->dataPtr->scanElem->GetElement("vertical");

  this->dataPtr->horzRayCount = this->RayCount();
  this->dataPtr->vertRayCount = this->VerticalRayCount();

  if (this->dataPtr->horzRayCount == 0 || this->dataPtr->vertRayCount == 0)
  {
    gzthrow("GpuSonarSensor: Image has 0 size!");
  }

  this->dataPtr->horzRangeCount = this->RangeCount();
  this->dataPtr->vertRangeCount = this->VerticalRangeCount();

  this->dataPtr->rangeMin = this->RangeMin();
  this->dataPtr->rangeMax = this->RangeMax();

  // Handle noise model settings.
  if (rayElem->HasElement("noise"))
  {
    this->noises[GPU_RAY_NOISE] =
        NoiseFactory::NewNoiseModel(rayElem->GetElement("noise"),
        this->Type());
  }

  this->dataPtr->parentEntity =
    this->world->EntityByName(this->ParentName());

  GZ_ASSERT(this->dataPtr->parentEntity != nullptr,
      "Unable to get the parent entity.");
}

//////////////////////////////////////////////////
void GpuSonarSensor::Init()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create GpuSonarSensor. Rendering is disabled.\n";
    return;
  }

  std::string worldName = this->world->Name();

  if (!worldName.empty())
  {
    this->scene = rendering::get_scene(worldName);

    if (!this->scene)
      this->scene = rendering::create_scene(worldName, false, true);

    this->dataPtr->sonarCam = this->scene->CreateGpuSonar(
        this->sdf->Get<std::string>("name"), false);

    if (!this->dataPtr->sonarCam)
    {
      gzerr << "Unable to create gpu sonar sensor\n";
      return;
    }
    this->dataPtr->sonarCam->SetCaptureData(true);

    // initialize GpuSonar from sdf
    // assume horizontal sweep (rotation around z axis) like cpu ray sensor
    this->dataPtr->sonarCam->SetIsHorizontal(true);

    this->dataPtr->sonarCam->SetNearClip(this->RangeMin());
    this->dataPtr->sonarCam->SetFarClip(this->RangeMax());

    // horizontal sonar setup
    double hfov = (this->AngleMax() - this->AngleMin()).Radian();

    if (hfov > 2 * M_PI)
    {
      hfov = 2 * M_PI;
      gzwarn << "Horizontal FOV for GPU sonar is capped at 180 degrees.\n";
    }

    this->dataPtr->sonarCam->SetHorzHalfAngle(
      (this->AngleMax() + this->AngleMin()).Radian() / 2.0);

    // determine number of cameras to use
    unsigned int cameraCount;
    if (hfov > 2.8)
    {
      if (hfov > 5.6)
      {
        cameraCount = 3;
      }
      else
      {
        cameraCount = 2;
      }
    }
    else
    {
      cameraCount = 1;
    }
    this->dataPtr->sonarCam->SetCameraCount(cameraCount);

    // horizontal fov of single frame
    hfov = hfov / cameraCount;

    this->dataPtr->sonarCam->SetHorzFOV(hfov);
    this->dataPtr->sonarCam->SetCosHorzFOV(hfov);

    // Fixed minimum resolution of texture to reduce steps in ranges
    // when hitting surfaces where the angle between ray and surface is small.
    // Also have to keep in mind the GPU's max. texture size
    unsigned int horzRangeCountPerCamera =
        std::max(2048U, this->dataPtr->horzRangeCount / cameraCount);
    unsigned int vertRangeCountPerCamera = this->dataPtr->vertRangeCount;

    // vertical sonar setup
    double vfov;

    if (this->dataPtr->vertRayCount > 1)
    {
      vfov = (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian();
    }
    else
    {
      vfov = 0;

      if (this->VerticalAngleMax() != this->VerticalAngleMin())
      {
        gzwarn << "Only one vertical ray but vertical min. and max. angle "
            "are not equal. Min. angle is used.\n";
        this->SetVerticalAngleMax(this->VerticalAngleMin().Radian());
      }
    }

    if (vfov > M_PI / 2)
    {
      vfov = M_PI / 2;
      gzwarn << "Vertical FOV for GPU sonar is capped at 90 degrees.\n";
    }

    this->dataPtr->sonarCam->SetVertFOV(vfov);
    this->dataPtr->sonarCam->SetVertHalfAngle((this->VerticalAngleMax()
                     + this->VerticalAngleMin()).Radian() / 2.0);

    this->SetVerticalAngleMin(this->dataPtr->sonarCam->VertHalfAngle() -
                              (vfov / 2));
    this->SetVerticalAngleMax(this->dataPtr->sonarCam->VertHalfAngle() +
                              (vfov / 2));

    // Assume camera always stays horizontally even if vert. half angle of
    // sonar is not 0. Add padding to camera vfov.
    double vfovCamera = vfov + 2 * std::abs(
        this->dataPtr->sonarCam->VertHalfAngle());

    // Add padding to vertical camera FOV to cover all possible rays
    // for given sonar vert. and horiz. FOV
    vfovCamera = 2 * atan(tan(vfovCamera / 2) / cos(hfov / 2));

    if (vfovCamera > 2.8)
    {
      gzerr << "Vertical FOV of internal camera exceeds 2.8 radians.\n";
    }

    this->dataPtr->sonarCam->SetCosVertFOV(vfovCamera);

    // If vertical ray is not 1 adjust horizontal and vertical
    // ray count to maintain aspect ratio
    if (this->dataPtr->vertRayCount > 1)
    {
      double cameraAspectRatio = tan(hfov / 2.0) / tan(vfovCamera / 2.0);

      this->dataPtr->sonarCam->SetRayCountRatio(cameraAspectRatio);
      this->dataPtr->rangeCountRatio = cameraAspectRatio;

      if ((horzRangeCountPerCamera / this->RangeCountRatio()) >
           vertRangeCountPerCamera)
      {
        vertRangeCountPerCamera =
            round(horzRangeCountPerCamera / this->RangeCountRatio());
      }
      else
      {
        horzRangeCountPerCamera =
            round(vertRangeCountPerCamera * this->RangeCountRatio());
      }
    }
    else
    {
      // In case of 1 vert. ray, set a very small vertical FOV for camera
      this->dataPtr->sonarCam->SetRayCountRatio(horzRangeCountPerCamera);
    }

    // Initialize camera sdf for GpuSonar
    this->dataPtr->cameraElem.reset(new sdf::Element);
    sdf::initFile("camera.sdf", this->dataPtr->cameraElem);

    this->dataPtr->cameraElem->GetElement("horizontal_fov")->Set(hfov);

    sdf::ElementPtr ptr = this->dataPtr->cameraElem->GetElement("image");
    ptr->GetElement("width")->Set(horzRangeCountPerCamera);
    ptr->GetElement("height")->Set(vertRangeCountPerCamera);
    ptr->GetElement("format")->Set("FLOAT32");

    ptr = this->dataPtr->cameraElem->GetElement("clip");
    ptr->GetElement("near")->Set(this->dataPtr->sonarCam->NearClip());
    ptr->GetElement("far")->Set(this->dataPtr->sonarCam->FarClip());

    // Load camera sdf for GpuSonar
    this->dataPtr->sonarCam->Load(this->dataPtr->cameraElem);

    // initialize GpuSonar
    this->dataPtr->sonarCam->Init();
    this->dataPtr->sonarCam->SetRangeCount(
        this->RangeCount(),
        this->VerticalRangeCount());
    this->dataPtr->sonarCam->SetClipDist(this->RangeMin(), this->RangeMax());
    this->dataPtr->sonarCam->CreateSonarTexture(
        this->ScopedName() + "_RttTex_Sonar");
    this->dataPtr->sonarCam->CreateRenderTexture(
        this->ScopedName() + "_RttTex_Image");
    this->dataPtr->sonarCam->SetWorldPose(this->pose);
    this->dataPtr->sonarCam->AttachToVisual(this->ParentId(), true, 0, 0);

    this->dataPtr->sonarMsg.mutable_scan()->set_frame(this->ParentName());
  }
  else
    gzerr << "No world name\n";

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
      ~rendering::Scene::GZ_SKYX_CLOUDS &
      ~rendering::Scene::GZ_SKYX_MOON);

  Sensor::Init();
}

//////////////////////////////////////////////////
void GpuSonarSensor::Fini()
{
  this->dataPtr->scanPub.reset();

  if (this->dataPtr->sonarCam)
  {
    this->scene->RemoveCamera(this->dataPtr->sonarCam->Name());
  }

  this->dataPtr->sonarCam.reset();

  Sensor::Fini();
}

//////////////////////////////////////////////////
event::ConnectionPtr GpuSonarSensor::ConnectNewSonarFrame(
  std::function<void(const float *, unsigned int, unsigned int, unsigned int,
  const std::string &)> _subscriber)
{
  return this->dataPtr->sonarCam->ConnectNewSonarFrame(_subscriber);
}

//////////////////////////////////////////////////
unsigned int GpuSonarSensor::CameraCount() const
{
  return this->dataPtr->sonarCam->CameraCount();
}

//////////////////////////////////////////////////
bool GpuSonarSensor::IsHorizontal() const
{
  return this->dataPtr->sonarCam->IsHorizontal();
}

//////////////////////////////////////////////////
double GpuSonarSensor::HorzFOV() const
{
  return this->dataPtr->sonarCam->HorzFOV();
}

//////////////////////////////////////////////////
double GpuSonarSensor::CosHorzFOV() const
{
  return this->dataPtr->sonarCam->CosHorzFOV();
}

//////////////////////////////////////////////////
double GpuSonarSensor::VertFOV() const
{
  return this->dataPtr->sonarCam->VertFOV();
}

//////////////////////////////////////////////////
double GpuSonarSensor::CosVertFOV() const
{
  return this->dataPtr->sonarCam->CosVertFOV();
}

//////////////////////////////////////////////////
double GpuSonarSensor::RayCountRatio() const
{
  return this->dataPtr->sonarCam->RayCountRatio();
}

//////////////////////////////////////////////////
double GpuSonarSensor::RangeCountRatio() const
{
  return this->dataPtr->rangeCountRatio;
}

//////////////////////////////////////////////////
ignition::math::Angle GpuSonarSensor::AngleMin() const
{
  return this->dataPtr->horzElem->Get<double>("min_angle");
}

//////////////////////////////////////////////////
void GpuSonarSensor::SetAngleMin(double _angle)
{
  this->dataPtr->horzElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle GpuSonarSensor::AngleMax() const
{
  return this->dataPtr->horzElem->Get<double>("max_angle");
}

//////////////////////////////////////////////////
void GpuSonarSensor::SetAngleMax(double _angle)
{
  this->dataPtr->horzElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
double GpuSonarSensor::RangeMin() const
{
  return this->dataPtr->rangeElem->Get<double>("min");
}

//////////////////////////////////////////////////
double GpuSonarSensor::RangeMax() const
{
  return this->dataPtr->rangeElem->Get<double>("max");
}

/////////////////////////////////////////////////
double GpuSonarSensor::AngleResolution() const
{
  return (this->AngleMax() - this->AngleMin()).Radian() /
    (this->RangeCount()-1);
}

//////////////////////////////////////////////////
double GpuSonarSensor::RangeResolution() const
{
  return this->dataPtr->rangeElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuSonarSensor::RayCount() const
{
  return this->dataPtr->horzElem->Get<unsigned int>("samples");
}

//////////////////////////////////////////////////
int GpuSonarSensor::RangeCount() const
{
  return this->RayCount() * this->dataPtr->horzElem->Get<double>("resolution");
}

//////////////////////////////////////////////////
int GpuSonarSensor::VerticalRayCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<unsigned int>("samples");
  else
    return 1;
}

//////////////////////////////////////////////////
int GpuSonarSensor::VerticalRangeCount() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
  {
    int rows =  (this->VerticalRayCount() *
          this->dataPtr->vertElem->Get<double>("resolution"));
    if (rows > 1)
      return rows;
    else
      return 1;
  }
  else
    return 1;
}

//////////////////////////////////////////////////
ignition::math::Angle GpuSonarSensor::VerticalAngleMin() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("min_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
void GpuSonarSensor::SetVerticalAngleMin(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("min_angle")->Set(_angle);
}

//////////////////////////////////////////////////
ignition::math::Angle GpuSonarSensor::VerticalAngleMax() const
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    return this->dataPtr->vertElem->Get<double>("max_angle");
  else
    return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
double GpuSonarSensor::VerticalAngleResolution() const
{
  return (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian() /
    (this->VerticalRangeCount()-1);
}

//////////////////////////////////////////////////
void GpuSonarSensor::SetVerticalAngleMax(const double _angle)
{
  if (this->dataPtr->scanElem->HasElement("vertical"))
    this->dataPtr->vertElem->GetElement("max_angle")->Set(_angle);
}

//////////////////////////////////////////////////
void GpuSonarSensor::Ranges(std::vector<double> &_ranges) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  _ranges.resize(this->dataPtr->sonarMsg.scan().ranges_size());
  memcpy(&_ranges[0], this->dataPtr->sonarMsg.scan().ranges().data(),
         sizeof(_ranges[0]) * this->dataPtr->sonarMsg.scan().ranges_size());
}

//////////////////////////////////////////////////
double GpuSonarSensor::Range(const int _index) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->sonarMsg.scan().ranges_size() == 0)
  {
    gzwarn << "ranges not constructed yet (zero sized)\n";
    return 0.0;
  }
  if (_index < 0 || _index > this->dataPtr->sonarMsg.scan().ranges_size())
  {
    gzerr << "Invalid range index[" << _index << "]\n";
    return 0.0;
  }

  return this->dataPtr->sonarMsg.scan().ranges(_index);
}

//////////////////////////////////////////////////
double GpuSonarSensor::Retro(const int /*_index*/) const
{
  return 0.0;
}

//////////////////////////////////////////////////
int GpuSonarSensor::Fiducial(const unsigned int /*_index*/) const
{
  return -1;
}

//////////////////////////////////////////////////
void GpuSonarSensor::Render()
{
  if (!this->dataPtr->sonarCam || !this->IsActive() || !this->NeedsUpdate())
    return;

  this->lastMeasurementTime = this->scene->SimTime();

  this->dataPtr->sonarCam->Render();
  this->dataPtr->rendered = true;
}

//////////////////////////////////////////////////
bool GpuSonarSensor::UpdateImpl(const bool /*_force*/)
{
  if (!this->dataPtr->rendered)
    return false;

  this->dataPtr->sonarCam->PostRender();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  msgs::Set(this->dataPtr->sonarMsg.mutable_time(),
      this->lastMeasurementTime);

  msgs::SonarScan *scan = this->dataPtr->sonarMsg.mutable_scan();

  // Store the latest sonar scans into sonarMsg
  msgs::Set(scan->mutable_world_pose(),
      this->pose + this->dataPtr->parentEntity->WorldPose());
  scan->set_angle_min(this->AngleMin().Radian());
  scan->set_angle_max(this->AngleMax().Radian());
  scan->set_angle_step(this->AngleResolution());
  scan->set_count(this->RangeCount());

  scan->set_vertical_angle_min(this->VerticalAngleMin().Radian());
  scan->set_vertical_angle_max(this->VerticalAngleMax().Radian());
  scan->set_vertical_angle_step(this->VerticalAngleResolution());
  scan->set_vertical_count(this->dataPtr->vertRangeCount);

  scan->set_range_min(this->dataPtr->rangeMin);
  scan->set_range_max(this->dataPtr->rangeMax);

  const int numRays = this->dataPtr->vertRangeCount *
    this->dataPtr->horzRangeCount;
  if (scan->ranges_size() != numRays)
  {
    // gzdbg << "Size mismatch; allocating memory\n";
    scan->clear_ranges();
    scan->clear_intensities();
    for (int i = 0; i < numRays; ++i)
    {
      scan->add_ranges(ignition::math::NAN_F);
      scan->add_intensities(ignition::math::NAN_F);
    }
  }

  auto dataIter = this->dataPtr->sonarCam->SonarDataBegin();
  auto dataEnd = this->dataPtr->sonarCam->SonarDataEnd();
  for (int i = 0; dataIter != dataEnd; ++dataIter, ++i)
  {
    const rendering::GpuSonarData data = *dataIter;
    double range = data.range;
    double intensity = data.intensity;

    // Mask ranges outside of min/max to +/- inf, as per REP 117
    if (range >= this->dataPtr->rangeMax)
    {
      range = ignition::math::INF_D;
    }
    else if (range <= this->dataPtr->rangeMin)
    {
      range = -ignition::math::INF_D;
    }
    else if (this->noises.find(GPU_RAY_NOISE) != this->noises.end())
    {
      range = this->noises[GPU_RAY_NOISE]->Apply(range);
      range = ignition::math::clamp(range,
          this->dataPtr->rangeMin, this->dataPtr->rangeMax);
    }

    range = ignition::math::isnan(range) ? this->dataPtr->rangeMax : range;
    scan->set_ranges(i, range);
    scan->set_intensities(i, intensity);
  }

  if (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections())
    this->dataPtr->scanPub->Publish(this->dataPtr->sonarMsg);

  this->dataPtr->rendered = false;

  return true;
}

//////////////////////////////////////////////////
bool GpuSonarSensor::IsActive() const
{
  return Sensor::IsActive() ||
    (this->dataPtr->scanPub && this->dataPtr->scanPub->HasConnections());
}

//////////////////////////////////////////////////
rendering::GpuSonarPtr GpuSonarSensor::SonarCamera() const
{
  return this->dataPtr->sonarCam;
}
