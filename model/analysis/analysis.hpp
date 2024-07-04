#ifndef ANALYSIS_HPP_INCLUDED
#define ANALYSIS_HPP_INCLUDED

#include <iostream>
#include <filesystem>
#include <deque>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <time.h>
#include <model/group.hpp>
#include <model/simulation.hpp>
#include <model/math.hpp>
#include "cvs_exporter.hpp"


#ifdef _WIN32
constexpr const char* RSCRIPT_BINARY = "RScript.exe";
#else
constexpr const char* RSCRIPT_BINARY = "rscript";
#endif


using namespace model;

namespace analysis
{
	using timeseries_t = std::deque<std::vector<float>>;
	using path_t = std::filesystem::path;


	template <typename Agent>
	inline float centrality(const Agent& pf, const size_t& idxf, const model::Simulation& sim)
	{
		vec3 adir(0.f);
		auto n = 0.f; // number of neighbors
		sim.visit_all<prey_tag>([&](auto& p, size_t idx) {
			if (idx != idxf) {
				if (sim.group_of<prey_tag>(idxf) == sim.group_of<prey_tag>(idx)) {
					adir += math::ofs(pf.pos, p.pos);
					++n;
				}	
			}
		});

		if (n) {
			return glm::length(adir / n);
		}
		return 0.f;
	}


	inline path_t output_path(const json& J)
	{
		auto exePath = std::filesystem::current_path();
		path_t top_folder = exePath;

		// PERHAPS CHANGE THE SAVING REPO - NOW GOES TO BIN

		//char* buf = nullptr;
		//size_t sz = 0;
		//if (_dupenv_s(&buf, &sz, "dancesPATH") == 0 && buf != nullptr)
		//{
		//	top_folder = buf;
		//	free(buf);
		//}

		std::string main_dat_folder = "sim_data";
		std::string outf = J["data_folder"];

		path_t filefolder = (top_folder / main_dat_folder).string();
		if (!(std::filesystem::exists(filefolder)))
		{
			std::filesystem::create_directory(filefolder.string());
		}
		filefolder = (filefolder / outf).string();
		if (!(std::filesystem::exists(filefolder)))
		{
			std::filesystem::create_directory(filefolder.string());
		}
		return filefolder;
	}

	inline path_t unique_output_folder(const json& J)
	{
		auto distr = std::uniform_int_distribution<int>(0, 1000);
		const auto random_id = std::to_string(distr(model::reng));
		const time_t now = time(0);
		struct tm* local_time = localtime(&now);

		const std::string thyear = std::to_string(1900 + local_time->tm_year);
		const std::string thmonth = std::to_string(1 + local_time->tm_mon) + std::to_string(local_time->tm_mday);
		const std::string thtime = std::to_string(local_time->tm_hour) + std::to_string(local_time->tm_min) + std::to_string(local_time->tm_sec);
		const std::string full_name = thyear + thmonth + thtime + std::to_string(now) + random_id;

		const auto outf = output_path(J);
		const path_t filefolder = (outf / full_name).string();
		if (!(std::filesystem::exists(filefolder))) {
			std::filesystem::create_directory(filefolder);
		}
		return filefolder;
	}

}
#endif
