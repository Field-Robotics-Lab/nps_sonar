#ifndef GAZEBO_RENDERING_GPUSONARRENDERTYPES_HH_
#define GAZEBO_RENDERING_GPUSONARRENDERTYPES_HH_

namespace gazebo
{
  namespace rendering
  {
    class GpuSonar;
    class SonarVisual;

    /// \def GpuSonarPtr
    /// \brief Shared pointer to GpuSonar
    typedef boost::shared_ptr<GpuSonar> GpuSonarPtr;

    /// \def SonarVisualPtr
    /// \brief Shared pointer to SonarVisual
    typedef std::shared_ptr<SonarVisual> SonarVisualPtr;
  }
}
#endif
