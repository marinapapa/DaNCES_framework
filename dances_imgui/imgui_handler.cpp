#include <implot.h>
#include <implot_internal.h>
#include <model/math.hpp>
#include <model/agents/agents.hpp>
#include "imgui_handler.hpp"
#include "imgui_unit.h"
#include "AppWin.h"
#include "imgui_styles.h"

namespace handler {

	class ImGuiDemo : public imgui_handler
	{
	public:
		ImGuiDemo(const json& J, bool enabled) { enabled_ = enabled; }
		std::string title() const override { return "ImGui Demo"; }

	protected:
		void do_display(const model::Simulation* sim) override {
			ImGui::ShowDemoWindow(&enabled_);
		}
	};


	class ImPlotDemo : public imgui_handler
	{
	public:
		ImPlotDemo(const json& J, bool enabled) { enabled_ = enabled; }
		std::string title() const override { return "ImPlot Demo"; }

	protected:
		void do_display(const model::Simulation* sim) override {
			ImPlot::ShowDemoWindow(&enabled_);
		}
	};


	class Tracking : public imgui_handler
	{
	public:
		Tracking(AppWin* appwin, const json& J, bool enabled) : appwin_(appwin) { enabled_ = enabled; }
		std::string title() const override { return "Tracking info"; }

	protected:
		void do_display(const model::Simulation* sim) override {
			auto* pip = appwin_->interactive_pip();
			if ((nullptr == pip) || (nullptr == pip->follow().pInstance)) {
				enabled_ = false; 
				return; 
			}
			auto& nir = pip->follow();
			auto* pInst = nir.pInstance;
			if (ImGui::Begin("Tracking", &enabled_)) {
				ImGui::Text("Camera mode: %s", CamModeNames[pip->mode()]);
				ImGui::Text("camera dist: %.2f m", pip->distance());
				ImGui::Separator();
				auto sc = ImGui::RadioButton("Prey", nir.species == 0);
				ImGui::SameLine();
				sc |= ImGui::RadioButton("Predator", nir.species == 1);
				auto sp = sc ? ((nir.species == 0) ? 1 : 0) : nir.species;
				auto idx = static_cast<int>(nir.idx);
				auto si = ImGui::InputInt("idx", &idx);
				if (sc || si) {
					pip->follow(appwin_, size_t(sp), size_t(idx));
				}
				ImGui::Text("pos:      %.2f %.2f %.2f", pInst->B[3].x, pInst->B[3].y, pInst->B[3].z);
				ImGui::Text("dir:      %.2f %.2f %.2f", pInst->B[0].x, pInst->B[0].y, pInst->B[0].z);
				ImGui::Text("side:     %.2f %.2f %.2f", pInst->B[2].x, pInst->B[2].y, pInst->B[2].z);
				ImGui::Text("speed:    %.2f m/s", pInst->speed);
				std::string sn = "unknown";
				glm::vec3 steering{ 0,0,0 };
				if (nir.species == model::prey_tag::value) {
					const auto& ind = sim->pop<model::prey_tag>()[nir.idx];
					sn = ind.get_current_state_descr();
					steering = ind.steering;
				}
				else if (nir.species == model::pred_tag::value) {
					const auto& ind = sim->pop<model::pred_tag>()[nir.idx];
					sn = ind.get_current_state_descr();
					steering = ind.steering;
				}
				ImGui::Text("state:    %s", sn.c_str());
				ImGui::Text("steering: %.2f %.2f %.2f", steering.x, steering.y, steering.z);
			}
			ImGui::End();
		}

	private:
		AppWin* appwin_ = nullptr;
	};


	class Groups : public imgui_handler
	{
	public:
		Groups(const json& J, bool enabled) { enabled_ = enabled; }
		std::string title() const override { return "Groups info"; }

	protected:
		void do_display(const model::Simulation* sim) override {
			if (ImGui::Begin("Groups", &enabled_)) {
				const auto& fd = sim->groups<prey_tag>();
				double n = std::accumulate(fd.cbegin(), fd.cend(), 0.0, [](double n, const auto& fi) { return n + static_cast<double>(fi.size); });
				ImGui::Text("Number of groups: %ld", fd.size());
				ImGui::Text("Mean size: %.1f", n / static_cast<double>(fd.size()));
			}
			ImGui::End();
		}
	};


	class ColorMapping : public imgui_handler
	{
	public:
		ColorMapping(AppWin* appwin, const json& J, bool enabled) : appwin_(appwin) {
			enabled_ = enabled;
			model::known_color_maps<model::prey_tag>::current = std::clamp(size_t(J["preys"]), size_t(0), model::known_color_maps<model::prey_tag>::size - 1);
			model::known_color_maps<model::pred_tag>::current = std::clamp(size_t(J["predator"]), size_t(0), model::known_color_maps<model::pred_tag>::size - 1);
		}
		std::string title() const override { return "Color Mapping"; }

	protected:
		void do_display(const model::Simulation* sim) override {
			if (ImGui::Begin("Color Mapping", &enabled_)) {
				auto* pip = appwin_->interactive_pip();
				auto dimm = pip->dimm();
				ImGui::SliderFloat("background dimming", &dimm, 0.f, 2.f);
				pip->dimm(dimm);
				std::array<bool, 2> trails{ pip->trail() };
				if (ImGui::CollapsingHeader("Prey")) {
					display_species<model::prey_tag>();
					ImGui::Checkbox("enable trail###Prey", &trails[0]);
				}
				if (ImGui::CollapsingHeader("Predator")) {
					display_species<model::pred_tag>();
					ImGui::Checkbox("enable trail###Predator", &trails[1]);
				}
				pip->trail(trails);
			}
			ImGui::End();
		}

	private:
		template <typename Tag>
		void display_species() {
			ImGui::PushID(Tag::value);  // we might have identical labels with different Tags
			const int nm = static_cast<int>(model::known_color_maps<Tag>::size);
			for (int i = 0; i < nm; ++i) {
				const char* descr = model::known_color_maps<Tag>::descr[i];
				ImGui::RadioButton(descr, &model::known_color_maps<Tag>::current, i);
				if ((i != nm-1) && (i + 1) % 3) { ImGui::SameLine(); }
			}
			ImGui::PopID();
		}

		AppWin* appwin_ = nullptr;
		std::array<bool, 2> disply_species_ = { false, false };
	};


	namespace gui {

		struct observables
		{
			std::vector<float> dist_from_roost_square;
			std::vector<float> mean_speed;
			std::vector<float> ind_speed;
			std::vector<float> ind_stress;
			std::vector<float> ind_y;
			std::vector<float> nnd;
			std::vector<float> time;
		};

		struct to_plot
		{
			bool speed_hist;
			bool altitude_hist;
			bool stress_hist;
			bool speed_timeseries;

			to_plot() :
				speed_hist(false),
				altitude_hist(false),
				stress_hist(false),
				speed_timeseries(false)
			{}
		};

	}


	template <typename Tag>
	class GuiObserver : public imgui_handler
	{
	public:
		GuiObserver(const json& J, bool enabled) { 
			enabled_ = enabled;
			ImguiThemeEditor_.ApplyTheme();
			roost_pos_plane_.x = J["roost_pos_plane"][0];
			roost_pos_plane_.y = J["roost_pos_plane"][1];
		}
		std::string title() const override { return "formerly known as GuiObserver"; }

	protected:

		void do_pre_display(const model::Simulation* sim) override
		{
				if (enabled_)
				{
						// Update whatever we want to show on the gui
						update_observables_(sim);
				}
	
		}

		void do_display(const model::Simulation* sim) override
		{
				if (enabled_) {
						const float pop_size = sim->pop<Tag>().size();
						const auto mean_alt = std::accumulate(to_show_.ind_y.begin(), to_show_.ind_y.end(), 0.f) / pop_size;
						const auto mean_nnd = std::accumulate(to_show_.nnd.begin(), to_show_.nnd.end(), 0.f) / pop_size;

						// Show Gui
						//ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 255, 0, 255));
						ImGui::Begin("GuiObserver here, at your service!", &enabled_);
						ImGui::Button("Press me, I do nothing.");
						ImGui::NewLine();
						ImGui::Text("Simulation time: %.2f s", to_show_.time.back());

						ImGui::NewLine();
						ImGui::Indent(0.1f);
						ImGui::Text("Mean speed is: %.2f m/s", to_show_.mean_speed.back());
						ImGui::Text("Mean NND is: %.2f m/s", mean_nnd);
						ImGui::Text("Mean distance from roost is: %.2f m", std::sqrt(to_show_.dist_from_roost_square.back()));
						ImGui::Text("Mean altitude: %.2f m", mean_alt);

						ImGui::NewLine();
						ImGui::Checkbox("Plot speed histogram?", &plot_.speed_hist); ImGui::SameLine();
						ImGui::Checkbox("Plot speed timeseries?", &plot_.speed_timeseries);
						ImGui::Checkbox("Plot stress histogram?", &plot_.stress_hist); ImGui::SameLine();
						ImGui::Checkbox("Plot altitude histogram?", &plot_.altitude_hist);

						if (plot_.speed_hist) { 
								auto p_name = "Speed (m/s)";
								auto p_title = "Speed Histogram";
								plot_ind_hist(to_show_.ind_speed, pop_size/2.f, 20, p_title, p_name, 10.0, 20.0, { 1.0, 0.0, 0.0, 0.5f });
						}
						if (plot_.altitude_hist) {
								auto p_name = "Y coord";
								auto p_title = "Altitude Histogram";
								plot_ind_hist(to_show_.ind_y, pop_size/2.f, 20, p_title, p_name, -10, 20, { 0.5, 0.1, 1.0, 0.8f });
						}
						if (plot_.stress_hist) {
								auto p_name = "Stress";
								auto p_title = "Stress Histogram";
								plot_ind_hist(to_show_.ind_stress, 10, 20, p_title, p_name, 0, 1, { 0.0, 2.0, 1.0, 0.8f });
						}
						if (plot_.speed_timeseries)
						{
								plot_speed_timeseries();
						}
						ImGui::End();
				}
		}


	private:

		void plot_ind_hist(
				const std::vector<float>& data_vec,
				const int& pop_size,
				const int& bins, 
				const char* plot_title,
				const char* bars_title,
				const double& xmin,
				const double& xmax,
				const ImVec4& col
		)
		{
			if (ImPlot::BeginPlot(plot_title)) {
				const auto* firstY = &*(data_vec.end() - pop_size);
				ImPlot::SetupAxesLimits(xmin, xmax, 0.0 , static_cast<double>(pop_size));
				ImPlot::SetNextFillStyle(col);
				ImPlot::PlotHistogram(bars_title, firstY, pop_size, bins);
				ImPlot::EndPlot();
			}
		}

		void plot_speed_timeseries()
		{
			int plot_depth = 300;
			if (to_show_.mean_speed.size() > plot_depth)
			{
				double speed_now = to_show_.mean_speed.back();
				double time_now = to_show_.time.back();

				if (ImPlot::BeginPlot("Speed")) {
					// std::vector iterators are basically pointers: 
					const auto* firstX = &*(to_show_.time.end() - plot_depth);
					const auto* firstY = &*(to_show_.mean_speed.end() - plot_depth);
					ImPlot::SetupAxesLimits(static_cast<double>(*firstX), time_now + 2, 6.0, 20.0, ImGuiCond_Always);
					ImPlot::PlotLine("Average speed", firstX, firstY, plot_depth);
					ImPlot::PlotScatter("Average speed", &time_now, &speed_now, 1);
					ImPlot::EndPlot();
				}
			}
			else {
				ImGui::Text("Collecting data, please wait...");
			}
		}

		
		void update_observables_(const model::Simulation* sim)
		{
			// A calculation for each observable 
			const auto& group = sim->pop<Tag>();
			const auto pop_size = group.size();

			//to_show_.ind_speed.resize(pop_size, 0.f);
			//to_show_.ind_stress.resize(pop_size, 0.f);
			//to_show_.nnd.resize(pop_size, 0.f);
			//to_show_.ind_y.resize(pop_size, 0.f);

			to_show_.ind_speed.clear();
			to_show_.ind_stress.clear();
			to_show_.nnd.clear();
			to_show_.ind_y.clear();

			float sum_speed = 0.f;
			sim->visit_all<Tag>([&](auto& p, size_t idx) {
				sum_speed += p.speed;
	/*			to_show_.ind_speed[idx] = p.speed;
				to_show_.ind_y[idx] = p.pos.y;
				to_show_.ind_stress[idx] = p.speed;*/

				to_show_.ind_speed.push_back(p.speed);
				to_show_.ind_y.push_back(p.pos.y);
				to_show_.ind_stress.push_back(p.stress);

				const auto& all_nb = sim->sorted_view<Tag>(idx); // all neighbors
				if (all_nb.size())	{
						to_show_.nnd.push_back(std::sqrt(all_nb.cbegin()->dist2));
				}
			});

			to_show_.mean_speed.push_back(sum_speed / pop_size);

			auto mean_ofs_center = glm::vec2(0.f);

			sim->visit_all<Tag>([&](auto& p, size_t idx) {
				mean_ofs_center += math::ofs(glm::vec2(p.pos.x, p.pos.z), roost_pos_plane_);
			});

			to_show_.dist_from_roost_square.push_back(glm::length2(mean_ofs_center) / pop_size);

			to_show_.time.push_back(sim->tick2time(sim->tick()));
		}

		personalized_imgui::ColorThemeEditor ImguiThemeEditor_;
		gui::to_plot plot_;
		gui::observables to_show_;
		glm::vec2 roost_pos_plane_ = { 0,0 };

	};


	class ScalarHistogram
	{
		using src_fun = std::function<double(const model::Prey&)>;

	public:
		ScalarHistogram(const json& jh, src_fun&& fun) : fun_(fun) {
			enabled_ = jh["enabled"];
			title_ = jh["type"];
		}

		bool& enabled() noexcept { return enabled_; }
		const std::string& title() { return title_; }

		void collect(const model::Simulation* sim) {
			data_.clear();
			for (const auto& ind : sim->pop<model::prey_tag>()) {
				data_.push_back(fun_(ind));
			}
		}
		
		void display(const model::Simulation* sim) {
			if ((enabled_ = ImGui::CollapsingHeader(title_.c_str()))) {
				if (ImPlot::BeginPlot("##Histograms")) {
					ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
					ImPlot::PlotHistogram("Empirical", data_.data(), data_.size(), ImPlotBin_Sturges, 1.0, {10,70});
					ImPlot::EndPlot();
				}
			}
		}
	
	private:
		src_fun fun_;
		std::vector<double> data_;
		bool enabled_;
		std::string title_;
	};


	ScalarHistogram create_histogram(const json& jh)
	{
		if (std::string(jh["type"]) == "Altitude") {
			return ScalarHistogram(jh, [](const model::Prey& prey) -> double {
				return prey.pos.y;
			});
		}
		else {
			throw std::runtime_error("unknown Histogram");
		}
	}


	class Histograms : public imgui_handler 
	{
	public:
		Histograms(const json& J, bool enabled) { 
			enabled_ = enabled;
			collect_dt_ = J["collect_interval"];
			const auto& js = J["sources"];
			for (size_t i = 0; i < js.size(); ++i) {
				hists_.emplace_back(create_histogram(js[i]));
			}
		}

		std::string title() const override { return "Histograms"; }

	protected:
		void do_pre_display(const model::Simulation* sim) override {
			const auto st = sim->tick2time(sim->tick());
			if (st > next_collect_) {
				for (auto& h : hists_) {
					if (h.enabled()) h.collect(sim);
				}
				next_collect_ += collect_dt_;
			}
		}

		void do_display(const model::Simulation* sim) override {
			if (ImGui::Begin("Histograms", &enabled_)) {
				for (auto& h : hists_) {
					h.display(sim);
				}
			}
			ImGui::End();
		}

	private:
		std::vector<ScalarHistogram> hists_;
		double next_collect_ = 0.0;
		double collect_dt_;			// [sim_time]
	};

}


std::unique_ptr<imgui_handler> create_imgui_handler(AppWin* appwin, const json& jh)
{
	std::string type = jh["type"];
	bool enabled = true;
	if ('~' == type[0]) {
		enabled = false;
		type.erase(type.begin());
	}
	if (type == "Tracking") return std::unique_ptr<imgui_handler>(new handler::Tracking(appwin, jh, enabled));
	else if (type == "Groups") return std::unique_ptr<imgui_handler>(new handler::Groups(jh, enabled));
	else if (type == "ColorMapping") return std::unique_ptr<imgui_handler>(new handler::ColorMapping(appwin, jh, enabled));
	else if (type == "GuiObserver") return std::unique_ptr<imgui_handler>(new handler::GuiObserver<model::prey_tag>(jh, enabled));
	else if (type == "Histograms") return std::unique_ptr<imgui_handler>(new handler::Histograms(jh, enabled));
	else if (type == "ImGui Demo") return std::unique_ptr<imgui_handler>(new handler::ImGuiDemo(jh, enabled));
	else if (type == "ImPlot Demo") return std::unique_ptr<imgui_handler>(new handler::ImPlotDemo(jh, enabled));
	else if (type == "ImGui Demo") return std::unique_ptr<imgui_handler>(new handler::ImGuiDemo(jh, enabled));
	else if (type == "ImPlot Demo") return std::unique_ptr<imgui_handler>(new handler::ImPlotDemo(jh, enabled));
	else {
		throw std::runtime_error("unknown imgui_handler");
	}
	return nullptr;
}
