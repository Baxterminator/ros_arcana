#include "arcana_cpp/lifecycle.hpp"
#include <lifecycle_msgs/msg/detail/transition__struct.hpp>

namespace arcana {

// ============================================================================
// Conversion functions
// ============================================================================

LCState_t rclcpp2arcana(const rcllc::State s) {
#define MAKE_CASE(u, v)                                                        \
  case rcllc::State::u:                                                        \
    return LCState::v;

  // Translate the two types
  switch (s.id) {
    MAKE_CASE(PRIMARY_STATE_UNKNOWN, UNKNOWN)
    MAKE_CASE(PRIMARY_STATE_UNCONFIGURED, UNCONFIGURED)
    MAKE_CASE(PRIMARY_STATE_INACTIVE, INACTIVE)
    MAKE_CASE(PRIMARY_STATE_ACTIVE, ACTIVE)
    MAKE_CASE(PRIMARY_STATE_FINALIZED, FINALIZE)
    MAKE_CASE(TRANSITION_STATE_CLEANINGUP, STEP_CLEANING)
    MAKE_CASE(TRANSITION_STATE_CONFIGURING, STEP_CONFIG)
    MAKE_CASE(TRANSITION_STATE_SHUTTINGDOWN, STEP_SHUTTING_DOWN)
    MAKE_CASE(TRANSITION_STATE_ACTIVATING, STEP_ACTIVATING)
    MAKE_CASE(TRANSITION_STATE_DEACTIVATING, STEP_DEACTIVATING)
    MAKE_CASE(TRANSITION_STATE_ERRORPROCESSING, STEP_ERROR_PROCESS)
  default:
    return LCState::LOST;
  }
#undef MAKE_CASE
}

constexpr LCState_t arcana2rclcpp(const LCState_t s) {
#define MAKE_CASE(u, v)                                                        \
  case LCState::v:                                                             \
    return rcllc::State::u;

  // Translate the two types
  switch (s) {
    MAKE_CASE(PRIMARY_STATE_UNCONFIGURED, UNCONFIGURED)
    MAKE_CASE(PRIMARY_STATE_INACTIVE, INACTIVE)
    MAKE_CASE(PRIMARY_STATE_ACTIVE, ACTIVE)
    MAKE_CASE(PRIMARY_STATE_FINALIZED, FINALIZE)
    MAKE_CASE(TRANSITION_STATE_CLEANINGUP, STEP_CLEANING)
    MAKE_CASE(TRANSITION_STATE_CONFIGURING, STEP_CONFIG)
    MAKE_CASE(TRANSITION_STATE_SHUTTINGDOWN, STEP_SHUTTING_DOWN)
    MAKE_CASE(TRANSITION_STATE_ACTIVATING, STEP_ACTIVATING)
    MAKE_CASE(TRANSITION_STATE_DEACTIVATING, STEP_DEACTIVATING)
    MAKE_CASE(TRANSITION_STATE_ERRORPROCESSING, STEP_ERROR_PROCESS)
  default:
    return rcllc::State::PRIMARY_STATE_UNKNOWN;
  }
#undef MAKE_CASE
}

constexpr const char *arcana2str(const LCState_t s) {
#define MAKE_CASE(v)                                                           \
  case LCState::v:                                                             \
    return #v;
  switch (s) {
    MAKE_CASE(UNKNOWN)
    MAKE_CASE(UNCONFIGURED)
    MAKE_CASE(INACTIVE)
    MAKE_CASE(ACTIVE)
    MAKE_CASE(FINALIZE)
    MAKE_CASE(STEP_CONFIG)
    MAKE_CASE(STEP_CLEANING)
    MAKE_CASE(STEP_SHUTTING_DOWN)
    MAKE_CASE(STEP_ACTIVATING)
    MAKE_CASE(STEP_DEACTIVATING)
    MAKE_CASE(STEP_ERROR_PROCESS)
    MAKE_CASE(LOST)
  }
#undef MAKE_CASE
}

// ============================================================================
// State navigation
// ============================================================================

//

Transition load_from_state(const LCState_t s) {
  switch (s) {
  case LCState::UNCONFIGURED:
  case LCState::STEP_CONFIG:
    return RCLTransition::TRANSITION_CONFIGURE;
  case LCState::INACTIVE:
  case LCState::STEP_ACTIVATING:
    return RCLTransition::TRANSITION_ACTIVATE;
  case LCState::ACTIVE:
    return RCLTransition::TRANSITION_ACTIVATE;

  // Else, return 255 (which will define as unknown transition code)
  case LCState::UNKNOWN:
  case LCState::STEP_ERROR_PROCESS:
  case LCState::LOST:
  case LCState::FINALIZE:
  case LCState::STEP_SHUTTING_DOWN:
  case LCState::STEP_CLEANING:
  case LCState::STEP_DEACTIVATING:
    return NO_TRANSITION;
  }
}

Transition unload_from_state(const LCState_t s) {
  switch (s) {
  case LCState::UNCONFIGURED:
  case LCState::STEP_SHUTTING_DOWN:
    return RCLTransition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  case LCState::INACTIVE:
  case LCState::STEP_CLEANING:
    return RCLTransition::TRANSITION_CLEANUP;
  case LCState::ACTIVE:
  case LCState::STEP_DEACTIVATING:
    return RCLTransition::TRANSITION_DEACTIVATE;
  case LCState::FINALIZE:
    return RCLTransition::TRANSITION_DESTROY;

  // Else, return 255 (which will define as unknown transition code)
  case LCState::UNKNOWN:
  case LCState::STEP_ERROR_PROCESS:
  case LCState::LOST:
  case LCState::STEP_CONFIG:
  case LCState::STEP_ACTIVATING:
    return NO_TRANSITION;
  }
}

Transition shutdown_from_state(const LCState_t s) {
  switch (s) {
  case LCState::UNCONFIGURED:
  case LCState::STEP_CLEANING:
    return RCLTransition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  case LCState::INACTIVE:
  case LCState::STEP_DEACTIVATING:
  case LCState::STEP_CONFIG:
    return RCLTransition::TRANSITION_INACTIVE_SHUTDOWN;
  case LCState::ACTIVE:
  case LCState::STEP_ACTIVATING:
    return RCLTransition::TRANSITION_ACTIVE_SHUTDOWN;

  case LCState::STEP_SHUTTING_DOWN:
  case LCState::UNKNOWN:
  case LCState::STEP_ERROR_PROCESS:
  case LCState::LOST:
  case LCState::FINALIZE:
    return 255;
  }
}

} // namespace arcana