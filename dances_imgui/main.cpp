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
    imgui_guard::init();
    auto clp = cmd::cmd_line_parser(argc, argv);
    auto project_dir = exe_path::get();
    if (clp.optional("project", project_dir)) {
      project_dir = std::filesystem::absolute(project_dir);
    }
    else if (!imgui_guard::gImgg()->headless()) {
      // ToDo: open file browser
    }
    if (!std::filesystem::is_directory(project_dir)) {
      throw std::runtime_error("can't open project directory");
    }

    // to not compose config but take pre-composed from command line
    std::filesystem::path arg_config = "";

    if ( clp.optional("config", arg_config )) {
        arg_config = std::filesystem::absolute(arg_config);
    } 

    auto J = compose_json(project_dir, arg_config);
    run(J);
    return 0;
  }
  catch (const std::exception& err) {
    std::cerr << err.what() << std::endl;
  }
  return -1;
}
