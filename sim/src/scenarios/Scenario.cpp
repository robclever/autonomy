#include <scenarios/Scenario.h>

namespace sim
{
namespace scenarios
{

Scenario::Scenario(World world, std::unique_ptr<Trajectory> trajectory, SensorSuite sensors,
                   systems::autonomy::SlamAlgorithm& slam, double dt)
    : world_(std::move(world)), trajectory_(std::move(trajectory)), sensors_(std::move(sensors)),
      slam_(slam), dt_(dt)
{
}

void Scenario::setUseAssociation(bool useAssoc) { useAssoc_ = useAssoc; }

void Scenario::run()
{
    while (!atEnd())
        step();
}

void Scenario::step()
{
    if (!atEnd())
    {
        ScenarioStep rec;
        rec.step = cursor_;
        rec.timeS = static_cast<double>(cursor_) * dt_;
        rec.truePose = trajectory_->poseAt(cursor_);

        /// @brief 1. Sense the world through all sensors.
        rec.detections = sensors_.sense(world_, rec.truePose);

        /// @brief 2. Convert detections to SLAM measurements.
        if (useAssoc_)
        {
            rec.measurements = MeasurementAdapter::convertAssociated(
                rec.detections, world_.landmarks(), rec.truePose);
        }
        else
        {
            rec.measurements = MeasurementAdapter::convert(rec.detections);
        }

        /// @brief 3. Run SLAM.
        rec.slamBefore = slam_.state();
        systems::autonomy::SlamInput input;
        input.pose = rec.truePose;
        input.measurements = rec.measurements;
        slam_.process(input);
        rec.slamAfter = slam_.state();

        steps_.push_back(std::move(rec));
        ++cursor_;
    }
}

void Scenario::reset()
{
    steps_.clear();
    cursor_ = 0;
    slam_.reset();
}

const std::vector<ScenarioStep>& Scenario::steps() const { return steps_; }

std::size_t Scenario::currentStep() const { return cursor_; }

bool Scenario::atEnd() const { return cursor_ >= trajectory_->numSteps(); }

} // namespace scenarios
} // namespace sim
