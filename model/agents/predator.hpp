#ifndef PRED_HPP_INCLUDED
#define PRED_HPP_INCLUDED

#include <math.hpp>
#include <agents/agents.hpp>
#include <states/transient.hpp>
#include <actions/align_actions.hpp>
#include <actions/cohere_actions.hpp>
#include <actions/mixed_actions.hpp>
#include <actions/cohere_speed_actions.hpp>
#include <actions/avoid_actions.hpp>
#include <actions/no_interacting_actions.hpp>
#include <actions/hunt_actions.hpp>
#include <stress/sources.hpp>
#include <stress/stress_base.hpp>
#include <states/transitions.hpp>
#include <model/flight_control.hpp>
#include <model/flight.hpp>
#include <model/json.hpp>


namespace model {
  
  template <>
  struct known_color_maps<pred_tag>
  {
    static constexpr size_t size = 4;
    static int current;
    static constexpr const char* descr[] = {
      "none   ",
      "speed  ",
      "state  ",
      "banking"
    };
    typedef float (*mapper) (const class Simulation& sim, const class Pred& agent, size_t idx);
    static std::array<mapper, size> fun;
  };


  template <>
  struct agent_instance<pred_tag>
  {
    vec3 pos = vec3(0);
    vec3 dir = vec3(0);

    static std::istream& stream_from_csv(std::istream& is, agent_instance<pred_tag>& e)
    {
      char delim;
      float discard;
      is >> discard >> delim; // discard id in local variable
      is >> e.pos.x >> delim >> e.pos.y >> delim;
      is >> e.dir.x >> delim >> e.dir.y >> delim >> e.dir.z;
      return is;
    }

    static std::ostream& stream_to_csv(std::ostream& os, const agent_instance<pred_tag>& e)
    {
      char delim[] = ", ";
      os << e.pos.x << delim << e.pos.y << delim;
      os << e.dir.x << delim << e.dir.y << delim << e.dir.y;
      return os;
    }
  };



  class Pred
  {
  public:
    using Tag = pred_tag;

    static constexpr const char* name() { return "Pred"; }

    using AP = states::package<
      states::transient<actions::package<
        Pred,
        actions::select_group<Pred>,
        actions::position_to_attack<Pred>
      >>,
      states::persistent<actions::package<
        Pred,
        actions::chase_closest_prey<Pred>
      >>,
      states::transient<actions::package<
        Pred,
        actions::set_retreat<Pred>
      >>,
      states::persistent<actions::package<Pred,
        actions::hold_current<Pred>
      >>
    >;
    using transitions = transitions::piecewise_linear_interpolator<AP::transition_matrix, 1>;

  public:
    Pred(Pred&&) = default;
    Pred(size_t idx, const json& J);
    void initialize(size_t idx, const Simulation& sim, const json& J);

    // returns next update time
    tick_t update(size_t idx, tick_t T, const Simulation& sim);
    void integrate(tick_t T, const Simulation& sim);
    void on_state_exit(size_t idx, tick_t T, const Simulation& sim);

    ::model::instance_proxy instance_proxy(size_t idx, const class Simulation* sim) const noexcept;
    ::model::agent_instance<Tag> get_instance(const Simulation* sim, size_t idx) const noexcept;
    void get_instance(Simulation* sim, size_t idx, const agent_instance<Tag>& se) noexcept;
    static std::vector<agent_instance<Tag>> init_pop(const Simulation& sim, const json& J);

    // unsynchronized queries used externally 
    const state_info_t& get_current_state() const noexcept { return current_state_; }
    size_t get_num_states() const noexcept { return AP::size; }
    std::string get_current_state_descr() const noexcept { return pa_[current_state_]->descr(); };

  public:
    vec3 pos = {};
    vec3 dir = {};
    head_system H;

    tick_t reaction_time = 0;   // [ticks]
    tick_t last_update = 0;
    tick_t copy_duration = 0;
    float speed = 0.f;            // [m/tick]
    vec3 accel = {};  // [m/tick ^ 2]
    vec3 steering = {};    // linear, lateral  [kg * m/tick^2]
    size_t target = -1; // idx of target
    tick_t state_timer;    // to be copyied by neighbors
    float stress; // prey need it
    flight::aero_info<float> ai;
	  flight::state_aero<float> sa;

  private:
    state_info_t current_state_;
    static transitions transitions_;
    AP::package_array pa_;
  };


}
#endif
