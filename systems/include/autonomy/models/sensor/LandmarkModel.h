#ifdef TODO

#ifndef AUTONOMY_MODELS_SENSOR_LANDMARKMODEL_H
#define AUTONOMY_MODELS_SENSOR_LANDMARKMODEL_H

#include <sensors/radar/RadarEcho.h>

#include <vector>

namespace systems::autonomy::models
{
// declaring relevant namespaces
using namespace systems::sensors::radar;

enum ProbabilityFunction
{
    GAUSSIAN,
    UNIFORM,
    CUSTOM
};

// Base class for potential landmark models.
// This is primarily used for range angle sensors, where landmarks cannot be uniquely identified.
// Assumes conditional independence of measurements in the given landmark model, which requires the
// noise in each individual measurement to be independent of the noise in other measurements. This
// allows us to process a feature at a time and utilize probalistic measurement models.
//
// Note that there are two types of maps, feature-based and location-based. Landmark meassurements
// are feature-based, which consists of lists of features.
class LandmarkModel
{
  public:
    LandmarkModel() = default;
    virtual ~LandmarkModel() = default;

    // Add a feature to the map. This is used to update the map with new features that are observed
    // by the sensor.
    virtual bool add_feature(const RadarEcho& feature);

    // Likelihood that a given measurement fits set, given the current state of the map. This is
    // used to compute the likelihood of a measurement given the current state of the map.
    virtual double compute_likelihood(const RadarEcho& measurement) const = 0;

    // Probability function for computation of likelihood
    virtual double set_probability_function() const = 0;

  private:
    // Map that consists of a list of features, which are the measurements that are observed by the
    // sensor.
    std::vector<RadarEcho> features_;
};

} // namespace systems::autonomy::models

#endif // AUTONOMY_MODELS_SENSOR_LANDMARKMODEL_H

#endif // TODO