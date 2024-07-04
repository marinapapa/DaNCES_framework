#pragma once

#include <string>
#include <filesystem>
#include <fstream>
#include <thread>
#include <regex>
#include <model/json.hpp>


namespace analysis {


  inline void append_bin2csv(std::filesystem::path path, size_t columns);

  
  inline std::string parse_header(const std::string& h) {
    if (h.empty()) return h;
    std::vector<std::string> cols;
    std::regex col_regex("([A-Za-z0-9_\\]\\[]+)");
    std::regex dup_regex("(.+)\\[([0-9]+)\\]");
    std::smatch m;
    std::string header;
    auto col_begin = std::sregex_iterator(h.begin(), h.end(), col_regex);
    for (auto it = std::sregex_iterator(h.begin(), h.end(), col_regex); it != std::sregex_iterator(); ++it) {
      const auto& col = it->str();
      if (std::regex_match(col, m, dup_regex)) {
        const auto dup = std::stoull(m[2].str());
        for (size_t i = 0; i < dup; ++i) {
          header += m[1].str() + std::to_string(i) + ',';
        }
      }
      else {
        header += col + ',';
      }
    }
    header.pop_back();  // remove trailing comma
    return header;
  }


  class cvs_exporter {
  public:
    cvs_exporter(const std::filesystem::path& out_path, const json& J) {
      skip_csv_ = J["skip_csv"];
      const std::string out_name = J["output_name"];
      header_ = parse_header(J["header"]);
      columns_ = std::count(header_.cbegin(), header_.cend(), ',') + 1;
      bin_path_ = out_path / (out_name + ".bin");
      os_.open(bin_path_, std::ios::binary);
      auto os = std::ofstream(std::filesystem::path(bin_path_).replace_extension(".csv"));
      os << header_ << '\n';
    }

    ~cvs_exporter() {
      if (os_ && !skip_csv_) {
        os_.close();
        append_bin2csv(bin_path_, columns_);
      }
    }

    void operator()(const float* first, size_t n) {
      os_.write((const char*)(first), sizeof(float) * n);
    }

    size_t columns() const noexcept { return columns_; }
    const std::string& header() const noexcept { return header_; }
    std::filesystem::path out_path() { return bin_path_.parent_path(); }

  private:
    std::ofstream os_;   // binary
    std::filesystem::path bin_path_;
    bool skip_csv_;
    size_t columns_;
    std::string header_;
  };


  inline void append_bin2csv(std::filesystem::path path, size_t columns) {
    auto is = std::ifstream(path, std::ios::binary);
    auto os = std::ofstream(std::filesystem::path(path).replace_extension(".csv"), std::ios::app);
    std::vector<float> row_data(columns);
    while (is) {
      // borderline UD
      is.read((char*)row_data.data(), sizeof(float) * columns);
      os << row_data[0];
      for (size_t c = 1; c < columns; ++c) {
        os << ',' << row_data[c];
      }
      os << '\n';
    }
  }

}
