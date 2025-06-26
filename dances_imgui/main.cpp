#include <filesystem>
#include <iostream>
#include <future>
#include <thread>
#include <tbb/tbb.h>
#include <tbb/global_control.h>
#include <model/json.hpp>
#include <model/model.hpp>
#include <libs/cmd_line.h>
#include "AppWin.h"
#include "analysis/meta_obs.hpp"


namespace headless {

  // model thread function
  void run_simulation(model::Simulation* sim,
                      const species_instances& ss,
                      model::Observer* observer,
                      const json& J)
  {
    auto Tmax = sim->time2tick(double(J["Simulation"]["Tmax"]));
    sim->initialize(observer, ss);
    while (!sim->terminated()) {
      sim->update(observer);
      if (sim->tick() == Tmax) {
        break;
      }
    }
    observer->notify(model::Simulation::Finished, *sim);
  }

}


// should come from file or something...
const static species_instances initial_snapshot = {
  {},                                  // prey
  {}                                   // predator(s)
};


void run(json& J)
{
  unsigned numThreads = J["Simulation"]["numThreads"];
  if (numThreads == -1) numThreads = std::thread::hardware_concurrency();
  numThreads = std::clamp(numThreads, 1u, std::thread::hardware_concurrency());
  tbb::global_control tbbgc(tbb::global_control::max_allowed_parallelism, numThreads);
  model::species_instances ss = initial_snapshot;
  auto sim = std::make_unique<model::Simulation>(J);
  auto observers = analysis::CreateObserverChain<model::prey_tag>(J);
  auto observer = std::make_unique<Observer>();
  std::for_each(observers.begin(), observers.end(), [&](const std::unique_ptr<Observer>& obs) {
    observer->append_observer(obs.get());
  });
  if (imgui_guard::gImgg()->headless()) {
    headless::run_simulation(sim.get(), ss, observer.get(), J);
  }
  else {
    AppWin appWin(imgui_guard::gImgg(), J);
    appWin.run_simulation(sim.get(), ss, observer.get(), J);
  }
}


int main(int argc, const char* argv[])
{
  try {
    auto clp = cmd::cmd_line_parser(argc, argv);
    auto project_dir = exe_path::get();
    if (clp.optional("project", project_dir)) {
      project_dir = std::filesystem::absolute(project_dir);
      if (!std::filesystem::is_directory(project_dir)) {
        throw std::runtime_error("can't open project directory");
      }
    }
    auto aconfig = std::string("config.json");
    clp.optional("config", aconfig);
    auto jconfig = from_path(project_dir / aconfig);
    if (!jconfig.has_value()) throw std::runtime_error("config json doesn't exist or corrupted");
    auto J = jconfig.value();

    auto inject = [&](const char* entry, std::string def) {
      if (clp.optional(entry, def)) {
        // mandatory cli override
        J[entry] = def;
      }
      if (J[entry].is_string()) {
        auto jo = from_path(project_dir / J[entry]);
        if (!jo.has_value()) throw std::runtime_error(std::string{entry} + " json missing or corrupt");
        J[entry] = jo.value()[entry];
      }
      if (!J[entry].is_object()) throw std::runtime_error(std::string("entry ") + entry + " missing w/o default");
    };
    inject("Prey", "./settings/prey.json");
    inject("Pred", "./settings/predator.json");
    inject("gui", "./imgui.json");
    inject("Simulation", "");

    // single value overrides
    if (clp.flag("--headless")) {
      J.at("gui").at("headless") = true;
    }
    size_t Tmax = -1;
    if (clp.optional("Tmax", Tmax)) {
      J.at("Simulation").at("Tmax") = Tmax;
    }
    if ((true == J.at("gui").at("headless")) && (J.at("Simulation").at("Tmax").get<size_t>() == size_t(-1))) {
      throw std::runtime_error("headless simulation would run forever");
    }
    imgui_guard::init(J);
    run(J);
    return 0;
  }
  catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
  }
  return -1;
}
