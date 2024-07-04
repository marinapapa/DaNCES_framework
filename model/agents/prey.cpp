#include <agents/prey.hpp>
#include <model/init_cond.hpp>


namespace model {

  namespace {
    thread_local rndutils::mutable_discrete_distribution<int, rndutils::all_zero_policy_uni> prey_discrete_dist;
  }

  // color mappings
  namespace prey_cm {
    float color_none(const Simulation& sim, const Prey& agent, size_t idx) { return 0.5f; }
    float color_idx(const Simulation& sim, const Prey& agent, size_t idx) { return float(idx) / sim.pop<prey_tag>().size(); }
    float color_speed(const Simulation& sim, const Prey& agent, size_t idx) { return agent.speed / agent.ai.maxSpeed; }
    float color_banking(const Simulation& sim, const Prey& agent, size_t idx) { return 0.5f + agent.H.beta() / math::pi<float>; }
    float color_state(const Simulation& sim, const Prey& agent, size_t idx) { return float(agent.get_current_state()) / agent.get_num_states(); }
    float color_substate(const Simulation& sim, const Prey& agent, size_t idx) {
      const auto nss = agent.get_num_substates();
      if (nss) {
        const auto ss = agent.get_current_state().sub_state();
        return static_cast<float>(ss + 1) / nss;
      }
      return 0.f;
    }
    float color_stress(const Simulation& sim, const Prey& agent, size_t idx) { return agent.stress / sim.pop<prey_tag>().size(); }
    float color_nnd2(const Simulation& sim, const Prey& agent, size_t idx) {
      const auto& all_nb = sim.sorted_view<prey_tag>(idx); // all neighbors
      if (all_nb.size()) {
        return all_nb.cbegin()->dist2;
      }
      return 0.5f;
    }
    float color_group(const Simulation& sim, const Prey& agent, size_t idx) {
      return float(sim.group_of<prey_tag>(idx)) / sim.groups<prey_tag>().size();
    }
  }

  int known_color_maps<prey_tag>::current = 0;

  std::array<known_color_maps<prey_tag>::mapper, known_color_maps<prey_tag>::size> known_color_maps<prey_tag>::fun = {
    &prey_cm::color_none,
    &prey_cm::color_idx,
    &prey_cm::color_speed,
    &prey_cm::color_banking,
    &prey_cm::color_state,
    &prey_cm::color_substate,
    &prey_cm::color_stress,
    &prey_cm::color_nnd2,
    &prey_cm::color_group
  };


  decltype(Prey::transitions_) Prey::transitions_;

  template <typename Init>
  void do_init_pop(std::vector<agent_instance<prey_tag>>& vse, Init&& init)
  {
    for (auto& e : vse) init(e);
  }


  std::vector<agent_instance<prey_tag>> Prey::init_pop(const Simulation& sim, const json& J)
  {
    const size_t N = J["N"];
    const auto& jic = J["InitCondit"];
    std::string type = jic["type"];
    if (type == "none") return {};
    std::vector<agent_instance<prey_tag>> vse(N);
    if (type == "random") do_init_pop(vse, initial_conditions::random(jic));
    else if (type == "flock") do_init_pop(vse, initial_conditions::in_flock(jic));
    else if (type == "csv") do_init_pop(vse, initial_conditions::from_csv(jic));
    else throw std::runtime_error("unknown initializer");
    return vse;
  }


  Prey::Prey(size_t idx, const json& J) :
    pos(0, 0, 0),
    dir(1, 0, 0),
    accel(0) // [m / s^2]
  {
    if (idx == 0) {
      transitions_ = decltype(transitions_)(J);
    }

    pa_ = AP::create(idx, J["states"]);
    stress_decay_ = J["stress"]["decay"]; // [stress/s]
    //float stress_mean = J["stress"]["ind_var_mean"]; 
    //float stress_sd = J["stress"]["ind_var_sd"]; 
    //if (stress_sd)
    //{
    //    auto str_pdist = std::normal_distribution<float>(stress_mean, stress_sd);
    //    stress = stress_ofs_ = str_pdist(model::reng);
    //}
    // else { 
    stress = stress_ofs_ = 0.f;
    // }
    sp_ = stress_accum::create(idx, J["stress"]["sources"]);

    ai = flight::create_aero_info<float>(J["aero"]);
    sa.w = 0.f; // until they get value from state (first integrates before update)
    speed = sa.cruiseSpeed = ai.cruiseSpeed;
  }

  void Prey::initialize(size_t idx, const Simulation& sim, const json& J)
  {
    H.initialize(*this);
    pa_[current_state_]->enter(this, idx, 0, sim, nullptr);
  }

  ::model::instance_proxy Prey::instance_proxy(size_t idx, const Simulation* sim) const noexcept
  {
    const float tex = known_color_maps<prey_tag>::fun[known_color_maps<prey_tag>::current](*sim, *this, idx);
    return { H.B(), speed, tex, 1.f, current_state_ };
  }

  ::model::agent_instance<prey_tag> Prey::get_instance(const Simulation* sim, size_t idx) const noexcept
  {
    return { pos, dir };
  }

  void Prey::get_instance(Simulation* sim, size_t idx, const agent_instance<prey_tag>& se) noexcept
  {
    pos = se.pos;
    dir = se.dir;
  }

  tick_t Prey::update(size_t idx, tick_t T, const Simulation& sim)
  {
    steering = vec3(0);
    pa_[current_state_]->resume(this, idx, T, sim);
    last_update = T;
    return T + reaction_time;
  }

  float Prey::assess_substates(size_t idx, tick_t T, const Simulation& sim, size_t state_idx, size_t sub_state_idx)
  {
      return pa_[state_idx]->assess_substate(this, idx, T, sim, sub_state_idx);
  }

  void Prey::integrate(tick_t T, const Simulation& sim)
  {
    flight_control::integrate_motion(this);
    if (stress > 0.f) { stress -= stress * (stress_decay_ * Simulation::dt()); }
    H.update(*this, sim.dt());
  }

  void Prey::on_state_exit(size_t idx, tick_t T, const Simulation& sim)
  {
    stress_accum::apply(sp_, this, idx, T, sim);
    if (copied_state.state() != current_state_.state()) {
      current_state_ = pa_[copied_state]->enter(this, idx, T, sim, &copied_state);
    }
    else {
      const auto TM = transitions_(stress);
      prey_discrete_dist.mutate(TM[current_state_].cbegin(), TM[current_state_].cend());
      auto next_state = prey_discrete_dist(reng);
      current_state_ = pa_[next_state]->enter(this, idx, T, sim, nullptr);
    }
    prev_exit_dir = dir;
    copied_state = current_state_;    // assume no copy
  }

}
