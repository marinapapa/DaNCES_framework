#ifndef META_OBS_HPP_INCLUDED
#define META_OBS_HPP_INCLUDED

#include <analysis/analysis.hpp>
#include <analysis/analysis_obs.hpp>
#include <model/observer.hpp>
#include <agents/agents.hpp>
#include <analysis/diffusion_obs.hpp>


namespace analysis
{
	
	template <typename Tag>
	std::vector<std::unique_ptr<Observer>> CreateObserverChain(json& J)
	{
		auto& ja = J["Simulation"]["Analysis"];
		auto& N = J["Prey"]["N"];
		std::vector<std::unique_ptr<Observer>> res;

		if (ja.size() == 0 || ja["data_folder"] == "")
		{
			std::cout << "No analysis observers created, data extraction will not take place." << std::endl;
			return res; // no observers created
		}
		const auto unique_path = analysis::unique_output_folder(ja);

		// inject output path to json object
		ja["output_path"] = unique_path.string();

		// copy json in output folder for reference
		save_json(J, (unique_path / J["Simulation"]["name"]));

		const auto& jo = ja["Observers"];
		for (const auto& j : jo)
		{
			std::string type = j["type"];
			if ('~' != type[0]) {
				if (type == "TimeSeries") res.emplace_back(std::make_unique<TimeSeriesObserver<Tag>>(unique_path, j));
				else if (type == "GroupData") res.emplace_back(std::make_unique<GroupObserver<Tag>>(unique_path, j));
				//else if (type == "NeighbData") res.emplace_back(std::make_unique<AllNeighborsObserver<Tag>>(unique_path, j, N));
				else if (type == "Diffusion") res.emplace_back(std::make_unique<DiffusionObserver<Tag>>(unique_path, j));
				else throw std::runtime_error("unknown observer");
			}
		}
		return res;
	}
}

#endif
