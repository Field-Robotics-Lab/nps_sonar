/*
Gazebo does not have plugins for dynamically adding sensor types
such as "gpu_sonar".  Gazebo statically adds its own sensor types such
as "gpu_ray" by defining hooks in them using "GZ_REGISTER_STATIC_SENSOR"
and then calling these hooks to instantiate their class.  Gazebo
stores the sensor type name and the instantiated class object in a Map.
When an SDF file presents a sensor type such as "gpu_ray" Gazebo finds
the class object in the Map and uses it.  If not, Gazebo quietly
drops the request.

We define a new sensor type and include a hook using Gazebo Macro
"GZ_REGISTER_STATIC_SENSOR" but we cannot statically include it without
modifying and recompiling Gazebo.  Instead, we create this system plugin
which calls the sensor type's hook for us during initialization to get
its class object into Gazebo's Map of plugins.  We define this system
plugin's hook using "GZ_REGISTER_SYSTEM_PLUGIN" and we get this system
plugin to run by providing parameter "-s <libsystem_plugin_name.so>".
*/

#include <gazebo/sensors/SensorFactory.hh> // for diagnostic
#include <gazebo/gazebo.hh>

extern void RegisterGpuSonarSensor();

namespace gazebo {

class AddGpuSonarSensor : public SystemPlugin {

    public: void Load(int _argc, char** _argv) {
        gzdbg <<"Adding GpuSonarSensor" << std::endl;
        RegisterGpuSonarSensor();

        // diagnostic
        std::vector<std::string> types;
        gazebo::sensors::SensorFactory::GetSensorTypes(types);

        for (const std::string& t : types) {
            gzdbg <<"Sensor type: \"" <<t <<"\"" <<std::endl;
        }
    }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(AddGpuSonarSensor);

};

