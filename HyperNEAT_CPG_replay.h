//
// Created by Gongjin Lan on 12/18/17.
//

#ifndef TOL_REVOLVE_HYPERNEAT_CPG_REPLAY_H
#define TOL_REVOLVE_HYPERNEAT_CPG_REPLAY_H

#include "HyperNEAT_CPG.h"

namespace tol
{
class HyperNEAT_CPG_replay : public HyperNEAT_CPG
{
public:
    HyperNEAT_CPG_replay(const std::string &_name,
                         const sdf::ElementPtr &_brain,
                         const EvaluatorPtr &_evaluator,
                         const std::vector<revolve::gazebo::MotorPtr> &_actuators,
                         const std::vector<revolve::gazebo::SensorPtr> &_sensors);

protected:
    void
    update(const std::vector<revolve::brain::ActuatorPtr> &actuators,
           const std::vector<revolve::brain::SensorPtr> &sensors,
           double t,
           double step) override;

    void
    update(const std::vector<rg::MotorPtr> &actuators,
           const std::vector<rg::SensorPtr> &sensors,
           double t,
           double step) override;


    boost::shared_ptr<cppneat::GeneticEncoding> offline_brain;
};
}

#endif //TOL_REVOLVE_HYPERNEAT_CPG_REPLAY_H
