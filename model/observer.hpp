#ifndef MODEL_OBSERVER_HPP_INCLUDED
#define MODEL_OBSERVER_HPP_INCLUDED

#include <vector>
#include <array>
#include <model/json.hpp>
#include "analysis/analysis.hpp"


// forward opaque type
class imgui_handler;


namespace model {

  class Observer
  {
  public:
    virtual ~Observer() {};
    virtual void notify(long long msg, const class Simulation& sim)
    {
      notify_next(msg, sim);
    }

    void notify_next(long long msg, const class Simulation& sim)
    {
      if (next_) next_->notify(msg, sim);
    }

    void append_observer(Observer* observer)
    {
      if (next_) next_->append_observer(observer);
      else next_ = observer;
    }

    virtual void notify_once(const class Simulation& sim)
    {
      if (next_) next_->notify_once(sim);
    }

    virtual imgui_handler* gui_handler() { return nullptr; }
    Observer* next() noexcept { return next_; }

  private:
    Observer* next_ = nullptr;
    Observer* parent_ = nullptr;
  };


  class AnalysisObserver : public model::Observer
  {
  public:
    size_t columns() const noexcept { return exporter_.columns(); }

    AnalysisObserver(const std::filesystem::path& out_path, const json& J) :
      exporter_(out_path, J) 
    {
        const double freq_sec = J["sample_freq"];
        oi_.sample_tick = oi_.sample_freq = static_cast<tick_t>(freq_sec / model::Simulation::dt());
        oi_.cached_rows = optional_json<size_t>(J, "cached_rows").value_or(10000);
    }
    virtual ~AnalysisObserver() {};

    struct obs_info
    {
      tick_t sample_freq;   // [ticks]
      tick_t sample_tick;
      size_t cached_rows;
    };

	  void notify(long long lmsg, const model::Simulation& sim)
	  {
		  using Msg = model::Simulation::Msg;
		  auto msg = Msg(lmsg);

		  switch (msg) {
		  case Msg::Tick: {
        notify_tick(sim);
			  if (sim.tick() >= oi_.sample_tick) {
				  notify_collect(sim);
				  oi_.sample_tick = sim.tick() + oi_.sample_freq;
			  }
			  if (data_out_.size() > columns() * oi_.cached_rows) // avoid overflow 
			  {
				  notify_save(sim);
				  data_out_.clear();
			  }
			  break;
		  }
      case Msg::PreTick: {
        notify_pre_tick(sim);
        if ((sim.tick() + 1) >= oi_.sample_tick) {
          notify_pre_collect(sim);
        }
        break;
      }
      case Msg::Initialized:
        notify_init(sim);
        break;
      case Msg::Finished:
			  notify_save(sim);
			  break;
      default:
        break;
		  }
		  notify_next(lmsg, sim);
	  }

  protected:
    virtual void notify_init(const model::Simulation&) {};
    virtual void notify_tick(const model::Simulation&) {};
    virtual void notify_pre_tick(const model::Simulation&) {};
    virtual void notify_collect(const model::Simulation&) {};
    virtual void notify_pre_collect(const model::Simulation&) {};
    virtual void notify_save(const model::Simulation&) {};
	   
  protected:
	  obs_info oi_;
	  std::vector<float> data_out_;
    analysis::cvs_exporter exporter_;
  };

}

#endif
