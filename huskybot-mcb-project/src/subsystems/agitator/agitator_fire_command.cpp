#include "agitator_fire_command.hpp"

namespace huskybot::subsystems::agitator
{
AgitatorFireCommand::AgitatorFireCommand(
    AgitatorSubsystem& agitatorSubsystem,
    algorithms::heat::HeatPredictor& heatPredictor)
    : agitator(agitatorSubsystem),
      heatPredictor(heatPredictor)
{
}

void AgitatorFireCommand::initialize()
{
    agitator.moveToNextPosition();
    heatPredictor.fireProjectile();
}

void AgitatorFireCommand::execute() {}

void AgitatorFireCommand::end(bool) {}

bool AgitatorFireCommand::isFinished() const { return agitator.atDesiredPosition(); }

bool AgitatorFireCommand::isReady() { return agitator.isOnline(); }
}  // namespace huskybot::subsystems::agitator