#include <agents/predator.hpp>
#include <model/init_cond.hpp>


namespace model {

  namespace {
    thread_local rndutils::mutable_discrete_distribution<int, rndutils::all_zero_policy_uni> pred_discrete_dist;
  }


  // color mappings
  namespace pred_cm {
    float color_none(const Simulation& sim, const Pred& agent, size_t idx) { return 1.f; }
    float color_speed(const Simulation& sim, const Pred& agent, size_t idx) { return agent.speed / agent.ai.maxSpeed; }
    float color_state(const Simulation& sim, const Pred& agent, size_t idx) { return float(agent.get_current_state()) / agent.get_num_states(); }
    float color_banking(const Simulation& sim, const Pred& agent, size_t idx) { return 0.5f + agent.H.beta() / math::pi<float>; }
  }

  int known_color_maps<pred_tag>::current = 0;

  std::array<known_color_maps<pred_tag>::mapper, known_color_maps<pred_tag>::size> known_color_maps<pred_tag>::fun = {
    &pred_cm::color_none,
    &pred_cm::color_speed,
    &pred_cm::color_state,
    &pred_cm::color_banking
  };


  decltype(Pred::transitions_) Pred::transitions_;
  // flight::aero_info<float> Pred::ai;
  //const flight::aero_info<float>& Pred::ai = Pred::ai;


  template <typename Init>
  void do_init_pop(std::vector<agent_instance<pred_tag>>& vse, Init&& init)
  {
    for (auto& e : vse) init(e);
  }  


  std::vector<agent_instance<pred_tag>> Pred::init_pop(const Simulation& sim, const json& J)
  {
    const size_t N = J["N"];
    auto jic = J["InitCondit"];
    std::string type = jic["type"];
    if (type == "none") return {};
    std::vector<agent_instance<pred_tag>> vse(N);
    if (type == "random") do_init_pop(vse, initial_conditions::random(jic));
    else if (type == "csv") do_init_pop(vse, initial_conditions::from_csv(jic));
    else throw std::runtime_error("unknown initializer");
    return vse;
  }



  Pred::Pred(size_t idx, const json& J) :
    state_timer(0), 
    pos(0, 0, 0),
    dir(1, 0, 0),
    accel(0) // [m / s^2]
  {
    if (idx == 0) {
      transitions_ = decltype(transitions_)(J);
    }
    ai = flight::create_aero_info<float>(J["aero"]);
    speed = sa.cruiseSpeed = ai.cruiseSpeed;
    sa.w = 0.f; // until they get value from state? (first integrates before update)
    pa_ = AP::create(idx, J["states"]);
  }

  void Pred::initialize(size_t idx, const Simulation& sim, const json& J)
  {
    H.initialize(*this);
    pa_[current_state_]->enter(this, idx, 0, sim, nullptr);
  }

  ::model::instance_proxy Pred::instance_proxy(size_t idx, const Simulation* sim) const noexcept
  {
    const float tex = known_color_maps<pred_tag>::fun[known_color_maps<pred_tag>::current](*sim, *this, idx);
    return { H.B(), speed, tex, 1.f, current_state_ };
  }

  ::model::agent_instance<pred_tag> Pred::get_instance(const Simulation* sim, size_t idx) const noexcept
  {
    return { pos,  dir };
  }

  void Pred::get_instance(Simulation* sim, size_t idx, const agent_instance<pred_tag>& se) noexcept
  {
    pos = se.pos;
    dir = se.dir;
  }

  tick_t Pred::update(size_t idx, tick_t T, const Simulation& sim)
  {
    steering = vec3(0);
    pa_[current_state_]->resume(this, idx, T, sim);
    last_update = T;
    return T + reaction_time;
  }

  void Pred::integrate(tick_t T, const Simulation& sim)
  {
    flight_control::integrate_motion(this);
    H.update(*this, sim.dt());
  }

  void Pred::on_state_exit(size_t idx, tick_t T, const Simulation& sim)
  {
    // select new state
    auto& dist = pred_discrete_dist;
    const auto TM = transitions_(0.f);
    pred_discrete_dist.mutate(TM[current_state_].cbegin(), TM[current_state_].cend());
    const auto next_state = pred_discrete_dist(reng);
    current_state_ = pa_[next_state]->enter(this, idx, T, sim, nullptr);
  }
}
