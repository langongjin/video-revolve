//
// Created by Gongjin Lan on 12/18/17.
//

#include <brain/Conversion.h>
#include "HyperNEAT_CPG_replay.h"
#include "BodyParser.h"
#include "Helper.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;


tol::HyperNEAT_CPG_replay::HyperNEAT_CPG_replay(const std::string &_name,
                                                const sdf::ElementPtr &_brain,
                                                const tol::EvaluatorPtr &_evaluator,
                                                const std::vector<revolve::gazebo::MotorPtr> &_actuators,
                                                const std::vector<revolve::gazebo::SensorPtr> &_sensors)

        : HyperNEAT_CPG(_name, _evaluator)
{
    // Initialise controller
    std::string name(_name.substr(0, _name.find('-')) + ".yaml");
    BodyParser body(name);

    std::tie(rb::InputMap, rb::OutputMap) = body.InputOutputMap(
            _actuators,
            _sensors);

    rb::RafCpgNetwork = rb::convertForController(body.CoupledCpgNetwork());
    rb::neuronCoordinates = body.IdToCoordinatesMap();

    // Initialise controller
    controller_ = rb::RafCPGControllerPtr(new rb::RafCPGController(
            _name,
            rb::RafCpgNetwork,
            Helper::createWrapper(_actuators),
            Helper::createWrapper(_sensors)));

    // Initialise learner
    auto learn_conf = parseLearningSDF(_brain);
    rb::SetBrainSpec(true);
    learn_conf.startFrom = body.CppnNetwork();
    cppneat::MutatorPtr mutator(new cppneat::Mutator(
            rb::brain_spec,
            0.8,
            learn_conf.startFrom->RangeInnovationNumbers().second,
            100,
            std::vector<cppneat::Neuron::Ntype>()));
    auto mutator_path =
            _brain->HasAttribute("path_to_mutator") ?
            _brain->GetAttribute("path_to_mutator")->GetAsString() : "none";

    // initialise learner
    auto learner_ = boost::shared_ptr<cppneat::NEATLearner>(
            new cppneat::NEATLearner(mutator, mutator_path, learn_conf));

    auto pathToFirstBrains =
            _brain->HasAttribute("path_to_first_brains") ?
            _brain->GetAttribute("path_to_first_brains")->GetAsString() : "";

    std::vector<cppneat::GeneticEncodingPtr> brainsFromFirst;

    if ("-" == pathToFirstBrains or "none" == pathToFirstBrains)
    {
        //ERROR
        throw std::runtime_error("Provide a brain to load!");
    }

    brainsFromFirst = boost::dynamic_pointer_cast<cppneat::NEATLearner>(
            learner_)->YamlBrains(pathToFirstBrains, -1);

    if (brainsFromFirst.size() <= 0)
    {
        //ERROR
        throw std::runtime_error("Brains list to load is emtpy");
    }

    this->offline_brain = brainsFromFirst.front();
}

void
tol::HyperNEAT_CPG_replay::update(const std::vector<rg::MotorPtr> &actuators, const std::vector<rg::SensorPtr> &sensors, double t, double step)
{
    HyperNEAT_CPG_replay::update(Helper::createWrapper(actuators),
                                 Helper::createWrapper(sensors), t,
                                 step);
}

void
tol::HyperNEAT_CPG_replay::update(const std::vector<revolve::brain::ActuatorPtr> &actuators, const std::vector<revolve::brain::SensorPtr> &sensors, double t, double step)
{
    if (isFirstRun_)
    {
        this->controller_->setPhenotype(
                convertForController_(this->offline_brain));

        startTime_ = t;
        evaluator_->start();
        isFirstRun_ = false;
    }

    // removed learner

    this->controller_->update(actuators, sensors, t, step);
}
