#include <scenarios/SensorSuite.h>

namespace sim
{
namespace scenarios
{

void SensorSuite::addDevice(std::unique_ptr<ISensorDevice> device)
{
    devices_.push_back(std::move(device));
}

std::vector<Detection> SensorSuite::sense(const World& world,
                                          const core::kinematics::Pose& aircraftPose) const
{
    std::vector<Detection> all;
    for (const auto& device : devices_)
    {
        const auto d = device->sense(world, aircraftPose);
        all.insert(all.end(), d.begin(), d.end());
    }
    return all;
}

std::size_t SensorSuite::deviceCount() const { return devices_.size(); }

} // namespace scenarios
} // namespace sim
