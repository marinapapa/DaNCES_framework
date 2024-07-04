#ifndef INIT_CONDIT_HPP_INCLUDED
#define INIT_CONDIT_HPP_INCLUDED

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <glmutils/random.hpp>
#include <model/math.hpp>
#include <model/simulation.hpp>


namespace initial_conditions {

   // config key: random
  class random
  {
  public:
	  random(const json& J) :
      radius_(J["radius"])
    {}

	  template <typename Instance>
	  void operator()(Instance& instance)
	  {
		  auto pdist = std::uniform_real_distribution<float>(0.f, radius_);
          instance.pos = model::vec3(pdist(model::reng), pdist(model::reng), pdist(model::reng));
          instance.dir = model::vec3(glmutils::unit_vec3(model::reng));
	  }

  private:
    float radius_;
  };

  
  // config key: csv
  class from_csv
  { 
  public:
	 from_csv(const json& J) :
		csv_(std::filesystem::path(std::string(J["file"])))
		{
			csv_.ignore(2048, '\n');    // skip header
		}

    template <typename Instance>
    void operator()(Instance& instance)
    {
	  // function reads only one line
        Instance::stream_from_csv(csv_, instance);
	  // delete line that has been read already
	    csv_.ignore(2048, '\n'); 
    }

  private:
    std::ifstream csv_;
  };


  // config key: flock
  class in_flock
  {
  public:
    in_flock(const json& J) :
          altitude_(J["pos"][1]),
		  dir0_(J["dir"][0], J["dir"][1], J["dir"][2]),
		  radius_(J["radius"]),
		  raddev_(glm::radians<float>(J["degdev"]))
	  {}

	  template <typename Instance>
	  void operator()(Instance& instance)
	  {
		  auto uni = std::uniform_real_distribution<float>(0.f, 1.f);
          instance.pos = radius_ * model::vec3(uni(model::reng), uni(model::reng), uni(model::reng)) + glm::vec3(0, altitude_, 0);
          const auto a = std::normal_distribution<float>(0, raddev_)(model::reng);
          auto Rz = glm::rotate(glm::mat4(1), a, glm::vec3(0, 1, 0));
          instance.dir = glm::vec3(Rz * glm::vec4(dir0_, 0.f));
	  }

  private:
	  model::vec3 dir0_;
    float altitude_;
	  float radius_;
	  float raddev_;
  };
}
#endif
