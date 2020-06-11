#ifndef NPS_BEAM_SENSOR_TYPES_HH
#define NPS_BEAM_SENSOR_TYPES_HH

namespace gazebo
{
  namespace sensors
  {
    class NpsBeamSensor;

    /// \def NpsBeamSensorPtr
    /// \brief Shared pointer to NpsBeamSensor
    typedef std::shared_ptr<NpsBeamSensor> NpsBeamSensorPtr;

    /// \def NpsBeamSensor_V
    /// \brief Vector of NpsBeamSensor shared pointers
    typedef std::vector<NpsBeamSensorPtr> NpsBeamSensor_V;
  }
}
#endif

