#ifndef GPU_SONAR_SENSOR_TYPES_HH
#define GPU_SONAR_SENSOR_TYPES_HH

namespace gazebo
{
  namespace sensors
  {
    class GpuSonarSensor;

    /// \def GpuSonarSensorPtr
    /// \brief Shared pointer to GpuSonarSensor
    typedef std::shared_ptr<GpuSonarSensor> GpuSonarSensorPtr;

    /// \def GpuSonarSensor_V
    /// \brief Vector of GpuSonarSensor shared pointers
    typedef std::vector<GpuSonarSensorPtr> GpuSonarSensor_V;
  }
}
#endif

