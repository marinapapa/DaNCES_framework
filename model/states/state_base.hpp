#ifndef MODEL_STATES_BASE_HPP_INCLUDED
#define MODEL_STATES_BASE_HPP_INCLUDED

#include <model/simulation.hpp>
#include <model/flight.hpp>


namespace model {

  namespace states {

    template <size_t I, typename Tuple, typename Elem>
    inline constexpr size_t tuple_idx() {
      if constexpr (std::is_same_v<Elem, std::tuple_element_t<I, Tuple>>) {
        return I;
      }
      else if constexpr ((I + 1) < std::tuple_size_v<Tuple>) {
        return tuple_idx<I + 1, Tuple, Elem>();
      }
      else {
        return -1;
      }
    };

    // a little tuple meta-function
    template <typename Package, typename Elem>
    inline constexpr size_t package_idx() { return tuple_idx<0, typename Package::package_tuple, Elem>(); }


    // abstract state
    template <typename Agent>
    class state
    {
    public:
      using agent_type = Agent;

      virtual ~state() = default;
      virtual state_info_t enter(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const state_info_t* copy_state) = 0;
      virtual float assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) = 0;
      virtual float assess_substate(agent_type* self, size_t idx, tick_t T, const Simulation& sim, const size_t substate_idx) = 0;
      virtual void resume(agent_type* self, size_t idx, tick_t T, const Simulation& sim) = 0;
      virtual bool is_copyable() const noexcept = 0;
      virtual std::string descr() const = 0;
      virtual size_t sub_states() const { return 0; }
    };


    template <typename ... States>
    class package
    {
    public:
      static constexpr size_t size = sizeof...(States);
      static_assert(size < state_info_t::max_idx, "way to many (sub) states");

      using package_tuple = std::tuple<States...>;
      using base_type = typename std::tuple_element_t<0, package_tuple>::base_type;
      using agent_type = typename base_type::agent_type;
      using package_array = std::array<std::unique_ptr<base_type>, size>;
      using transition_matrix = std::array<std::array<float, size>, size>;

      static package_array create(size_t idx, const json& J)
      {
        package_array a;
        if (J.size() != size) throw std::runtime_error("Parsing error: Number of states differs in code and config  \n");
        do_create<0>::apply(a, idx, J);
        return a;
      }

    private:
      template <size_t I>
      struct do_create
      {
        template <typename Json>
        static void apply(package_array& a, size_t idx, const Json& J)
        {
          using type = std::tuple_element_t<I, package_tuple>;
          //assert(J[I]["name"] == type::name());
          std::string n = J[I]["name"];
          if (J[I]["name"] != type::name()) throw std::runtime_error("Parsing error: Name of state differs in code (" + std::string(type::name()) + ") and config (" + std::string(J[I]["name"])+ ")  \n");
          a[I].reset(new type(I, idx, J[I]));
          if constexpr (I < size - 1) {
            do_create<I + 1>::apply(a, idx, J);
          }
        }
      };
    };
  }
}


// a: state-name
#define make_state_from_this(a) \
public: \
  using agent_type = typename IP::agent_type; \
  using base_type = state<agent_type>; \
  static constexpr const char* name() noexcept { return #a; } \
  std::string descr() const override { return descr_; } \
protected: \
  using action_pack = IP; \
  using action_tuple = typename action_pack::package_tuple; \
  action_tuple actions; \
  float all_ws = 0.f; \
  template <size_t I> \
  void chain_actions(agent_type* self, size_t idx, tick_t T, const Simulation& sim) { \
    std::get<I>(actions)(self, idx, T, sim); \
    if constexpr (I < action_pack::size - 1) chain_actions<I + 1>(self, idx, T, sim); \
  } \
  template <size_t I> \
  void chain_on_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) { \
    std::get<I>(actions).on_entry(self, idx, T, sim); \
    if constexpr (I < action_pack::size - 1) chain_on_entry<I + 1>(self, idx, T, sim); \
  } \
  template <size_t I> \
  void normalize_actions() { \
      all_ws += std::get<I>(actions).w_; \
      normalize_actions<I + 1>(); \
      if (all_ws > 0) { if constexpr (I < action_pack::size - 1) std::get<I>(actions).w_ /= all_ws; } \
  } \
  template <size_t I> \
  void chain_assess_entry(agent_type* self, size_t idx, tick_t T, const Simulation& sim) { \
    actions_potential_.push_back(std::get<I>(actions).assess_entry(self, idx, T, sim)); \
    if constexpr (I < action_pack::size - 1) chain_assess_entry<I + 1>(self, idx, T, sim); \
  } \
private: \
  bool is_copyable() const noexcept { return copyable_; } \
  std::string descr_; \
  bool copyable_ = false; \


#endif
