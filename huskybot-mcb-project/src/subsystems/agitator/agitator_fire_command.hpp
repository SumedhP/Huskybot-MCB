#pragma once

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "algorithms/heat/heat_predictor.hpp"

#include "agitator_subsystem.hpp"

namespace huskybot::subsystems::agitator
{
class AgitatorFireCommand : public tap::control::Command
{
public:
    AgitatorFireCommand(
        AgitatorSubsystem& agitatorSubsystem,
        algorithms::heat::HeatPredictor& heatPredictor);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "Agitator Fire Command"; }

private:
    AgitatorSubsystem& agitator;
    algorithms::heat::HeatPredictor& heatPredictor;
};
}  // namespace huskybot::subsystems::agitator