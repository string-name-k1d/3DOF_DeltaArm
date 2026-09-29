#include "arm/arm_state_control.hpp"

namespace DeltaArmRos {

void ArmStateControl::configureHome(const std::array<double, 3> & home_m) { home_m_ = home_m; }

const char * ArmStateControl::toString(State state) noexcept {
    switch (state) {
    case State::kZeroEmergency:
        return "zero_emergency";
    case State::kHome:
        return "home";
    case State::kPositionControl:
        return "position_control";
    }
    return "unknown";
}

void ArmStateControl::set(State state, const std::string & reason) {
    state_ = state;
    last_reason_ = reason;
    ++transitions_;
}

bool ArmStateControl::requestState(State state, const std::string & reason) {
    if (state == State::kZeroEmergency) {
        return activateEmergencyStop(reason.empty() ? "requested zero state" : reason);
    }
    // Guard: never silently clear a latched estop via requestState.
    if (isEmergencyActive()) {
        return false;
    }
    if (state == state_) {
        return false;
    }
    set(state, reason.empty() ? std::string("requested state change") : reason);
    return true;
}

bool ArmStateControl::activateEmergencyStop(const std::string & reason) {
    if (isEmergencyActive()) {
        last_reason_ = reason.empty() ? last_reason_ : reason;
        return false;
    }
    set(State::kZeroEmergency, reason.empty() ? "emergency stop activated" : reason);
    return true;
}

bool ArmStateControl::releaseEmergencyStop(const std::string & reason) {
    if (!isEmergencyActive()) {
        return false;
    }
    set(State::kHome, reason.empty() ? "emergency stop released" : reason);
    return true;
}

bool ArmStateControl::onGoalAccepted() {
    if (!acceptsGoals()) {
        return false;
    }
    if (state_ == State::kPositionControl) {
        return false;
    }
    set(State::kPositionControl, "position goal accepted");
    return true;
}

bool ArmStateControl::onHomeReached() {
    if (!acceptsGoals()) {
        return false;
    }
    if (state_ == State::kHome) {
        return false;
    }
    set(State::kHome, "home posture reached");
    return true;
}

}  // namespace DeltaArmRos
