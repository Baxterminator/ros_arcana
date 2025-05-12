#ifndef ARCANA_LIFECYCLE_STATE_HPP
#define ARCANA_LIFECYCLE_STATE_HPP

/*
  =============================================================================
                          Arcana - LifeCycle states

  This file describe the states of the lifecycle nodes.

  Author: Geoffrey "Meltwin" Côte (https://github.com/Meltwin)
  Copyright: (c) Meltwin - 2025
  Distributed under the MIT Licence
  =============================================================================
*/

#include <lifecycle_msgs/msg/detail/state__struct.hpp>
#include <lifecycle_msgs/msg/detail/transition__struct.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace rcllc = lifecycle_msgs::msg;

namespace arcana {

// ============================================================================
// State enumeration
// ============================================================================

typedef rcllc::State::_id_type LCState_t;

/**
  @brief Mirror structure of the lifecycle states of the rclcpp_lifecycle
  library.
*/
struct LCState {
  enum _states : LCState_t {
    UNKNOWN = 0u,
    FINALIZE = 1u,
    STEP_ERROR_PROCESS = 2u,
    STEP_SHUTTING_DOWN = 3u,
    UNCONFIGURED = 4u,
    STEP_CONFIG = 5u,
    STEP_CLEANING = 6u,
    INACTIVE = 7u,
    STEP_DEACTIVATING = 8u,
    STEP_ACTIVATING = 9u,
    ACTIVE = 10u,
    LOST = 255u
  };
};

typedef rcllc::Transition RCLTransition;
typedef RCLTransition::_id_type Transition;
constexpr Transition NO_TRANSITION{255};

// ============================================================================
// State navigation
// ============================================================================

/**
  @brief Get the transition to use to load from this state (i.e. going to a
  more active state).
  If no transition is available, this function returns 255.
*/
Transition load_from_state(const LCState_t s);

/**
  @brief Get the transition to use to unload from this state (i.e. going to a
  more inactive state). If the state is UNCONFIGURED, goes to FINALIZED.
  If no transition is available, this function returns 255.
*/
Transition unload_from_state(const LCState_t s);

/**
  @brief Get the transition to use to shutdown from this state (i.e. going to
  the finalize state).
*/
Transition shutdown_from_state(const LCState_t s);

// ============================================================================
// Conversion functions
// ============================================================================
LCState_t rclcpp2arcana(const rcllc::State s);
constexpr LCState_t arcana2rclcpp(const LCState_t s);
constexpr const char *arcana2str(const LCState_t s);

} // namespace arcana

#endif