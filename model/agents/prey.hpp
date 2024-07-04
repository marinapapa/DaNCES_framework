#ifndef PREY_HPP_INCLUDED
#define PREY_HPP_INCLUDED

#include <istream>
#include <ostream>
#include <optional>
#include <model/json.hpp>
#include <model/math.hpp>
#include <glmutils/random.hpp>
#include <glm/glm.hpp>
#include <states/transient.hpp>
#include <states/persistent.hpp>
#include <states/multi_state.hpp>
#include <actions/align_actions.hpp>
#include <actions/cohere_actions.hpp>
#include <actions/mixed_actions.hpp>
#include <actions/cohere_speed_actions.hpp>
#include <actions/avoid_actions.hpp>
#include <actions/predator_actions.hpp>
#include <actions/avoid_pred_actions.hpp>
#include <actions/escape_actions.hpp>
#include <actions/no_interacting_actions.hpp>
#include <actions/roosting.hpp>
#include <actions/flight_actions.hpp>
#include <stress/sources.hpp>
#include <stress/stress_base.hpp>
#include <states/transitions.hpp>
#include <model/flight_control.hpp>
#include <model/flight.hpp>

namespace model {

  template <>
  struct known_color_maps<prey_tag>
  {
    static constexpr size_t size = 9;
    static int current;
    static constexpr const char* descr[size] = {
      "none    ",
      "idx     ",
      "speed   ",
      "banking ",
      "state   ",
      "substate",
      "stress  ",
      "nnd2    ",
      "group   "
    };
    typedef float (*mapper) (const class Simulation& sim, const class Prey& agent, size_t idx);
    static std::array<mapper, size> fun;
  };


  template <>
  struct agent_instance<prey_tag>
  {
    vec3 pos = vec3(0);
    vec3 dir = vec3(0);

    static std::istream& stream_from_csv(std::istream& is, agent_instance<prey_tag>& e)
    {
      char delim;
      float discard;
      is >> discard >> delim; // discard id in local variable
      is >> e.pos.x >> delim >> e.pos.y >> delim >> e.pos.z >> delim;
      is >> e.dir.x >> delim >> e.dir.y >> delim >> e.dir.z;
      return is;
    }

    static std::ostream& stream_to_csv(std::ostream& os, const agent_instance<prey_tag>& e)
    {
      char delim = ',';
      os << e.pos.x << delim << e.pos.y << delim << e.pos.z << delim;
      os << e.dir.x << delim << e.dir.y << delim << e.dir.z;
      return os;
    }
  };


  class Prey
  {
  public:
    using Tag = prey_tag;

    static constexpr const char* name() { return "Prey"; }

    // order of escape states not trivial (selector depends on it)
    using flee_state = states::multi_state<
      Prey,
      states::persistent<actions::package<
        Prey, // escape turn
        actions::random_t_turn_gamma_pred<Prey>,
        actions::wiggle<Prey>
      >>,
      states::persistent<actions::package<
        Prey, // dive
        actions::dive<Prey>
      >>
    >;

    // order of states not trivial (transition matrix depends on it)
    using AP = states::package<
      states::transient<actions::package<
        Prey, // 1. regular flocking
        actions::align_n<Prey>,
		    actions::cohere_centroid_distance<Prey>,
        actions::avoid_n_position<Prey>,
        actions::altitude_attraction<Prey>,
        actions::copy_escape<Prey>, // with copy escape
        actions::wiggle<Prey>
      >>, 
      flee_state, // 2. escape multistate
      states::persistent <actions::package<
        Prey, // 3. refraction - persistent flocking
        actions::align_n<Prey>,
        actions::cohere_centroid_distance<Prey>,
        actions::avoid_n_position<Prey>,
        actions::roost_attraction<Prey>,
        actions::altitude_attraction<Prey>,
        actions::level_attraction<Prey>,
        actions::wiggle<Prey>
      >>
    >;
    using stress_accum = stress::accumulator<Prey,
      stress::predator_distance<Prey>
    >;
    using transitions = transitions::piecewise_linear_interpolator<AP::transition_matrix, 3>; // based on transition cuts of interpolation

  public:
    Prey(Prey&&) = default;
    Prey(size_t idx, const json& J);

    void initialize(size_t idx, const Simulation& sim, const json& J);

    // returns next update time
    tick_t update(size_t idx, tick_t T, const Simulation& sim);
    void integrate(tick_t T, const Simulation& sim);
    void on_state_exit(size_t idx, tick_t T, const Simulation& sim);
    float assess_substates(size_t idx, tick_t T, const Simulation& sim, size_t state_idx, size_t sub_state_idx);

    ::model::instance_proxy instance_proxy(size_t idx, const Simulation* sim) const noexcept;
    ::model::agent_instance<Tag> get_instance(const Simulation* sim, size_t idx) const noexcept;
    void get_instance(Simulation* sim, size_t idx, const agent_instance<Tag>& se) noexcept;
  //  float assess_current_state(size_t idx, const Simulation* sim) const noexcept { return pa_[current_state_.state()]->assess_substate(this, idx, T, sim, i);; };

    // unsynchronized queries used externally
    const state_info_t& get_current_state() const noexcept { return current_state_; }
    size_t get_num_states() const noexcept { return AP::size; }
    size_t get_num_substates() const noexcept { return pa_[current_state_]->sub_states(); }
    std::string get_current_state_descr() const noexcept { return pa_[current_state_]->descr(); };

  public:
    // accessible from states
    vec3 pos = {};   // [m]
    vec3 dir = {};
    head_system H;

    float speed;  // [m/tick]
    vec3 accel = {};  // [m/tick ^ 2]
    tick_t reaction_time = 0;   // [ticks]
    tick_t last_update = 0; 
    float stress;
    std::array<float, AP::size> tm; // transition matrix line per state change evaluation for export
    //vec3 force = {};              // reserved for physical forces  [kg * m/tick^2]
    vec3 steering = {};             // linear, lateral  [kg * m/tick^2]
    state_info_t copied_state;
    glm::vec3 prev_exit_dir;        // direction at previous state-exit

    flight::aero_info<float> ai;
    flight::state_aero<float> sa;
    static std::vector<agent_instance<Tag>> init_pop(const Simulation& sim, const json& J);

  private:
    state_info_t current_state_;
    static transitions transitions_;
    float stress_ofs_; // stress offset (individual variation)
    float stress_decay_; // same of all prey

    AP::package_array pa_;
    typename stress_accum::package_tuple sp_;
  };

  namespace states {

    // Specialization for Prey of the sub_state-selector(s)
    template <>
    struct sub_state_selector<Prey>
    {
      explicit sub_state_selector(const json J) {
        probs = J["selector"]["probs"];
        override_probs = J["selector"]["override_from_actions"];
      }

      size_t operator()(typename Prey::flee_state& multi_state, Prey* self, size_t idx, tick_t T, const Simulation& sim, const size_t state_idx ) {
        using multi_state_t = typename Prey::flee_state;

        if (override_probs) {
            std::array<float, multi_state_t::num_substates()> sprobs;
            for (size_t i = 0; i < multi_state_t::num_substates(); i++)
            {
                probs[i] = self->assess_substates(idx, T, sim, state_idx, i);
            }
        }    

        std::array<float, multi_state_t::num_substates()> sprobs(probs);
        selector_discrete_dist.mutate(probs.cbegin(), probs.cend());
        auto escape_state = selector_discrete_dist(reng);
        return escape_state;
      }

      // it is possible to have multiple specializations
      // size_t operator()(typename Prey::other_multi_state& ...)

      rndutils::mutable_discrete_distribution<int, rndutils::all_zero_policy_uni> selector_discrete_dist;
      std::array<float, 2> probs; // TO FIX, needs to be manually changed when changing number of escapes
      bool override_probs;
    };

  }


}
#endif
