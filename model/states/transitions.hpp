#ifndef MODEL_TRANSITIONS_HPP_INCLUDED
#define MODEL_TRANSITIONS_HPP_INCLUDED

#include <iostream>
#include <model/json.hpp>

namespace model {
  namespace transitions {

	  // constant transition matrix
	  template <typename TM, size_t I>
	  class constant
	  {
	  public:
		  constant() = default;
		  constant(const json& J)
		  {
			  const json& jt = J["transitions"];
			  //assert(std::string(jt["name"]) == "piecewise_linear_interpolator");
			  if (jt["name"] != "constant") throw std::runtime_error("Parsing error: Name of transition interpolator wrong in config (" + std::string(jt["name"]) + ")\n");

			  //assert(I == jt["TM"].size());
			  if (I != jt["TM"].size()) throw std::runtime_error("Parsing error: size of transition matrix wrong in config \n");
			  TM_ = jt["TM"];
		  }

		  TM operator()() const
		  {
			return TM_[0];
		  }

	  private:
		  std::array<TM, I> TM_;
	  };

    // piecewise linear interpolation between I
    // transition matrices. works with single matrix
    // (I == 1) for convenience.
    template <typename TM, size_t I>
    class piecewise_linear_interpolator
    {
    public:
      piecewise_linear_interpolator() = default;
      piecewise_linear_interpolator(const json& J)
      {
        const json& jt = J.at("transitions");
        //assert(std::string(jt["name"]) == "piecewise_linear_interpolator");
        if (jt.at("name") != "piecewise_linear_interpolator") throw std::runtime_error("Parsing error: Name of transition interpolator wrong in config (" + std::string(jt.at("name")) + ")\n");

        if (I != jt.at("TM").size()) throw std::runtime_error("Parsing error: size of transition matrix wrong in config \n");

        if (I != jt.at("edges").size()) throw std::runtime_error("Parsing error: number of transition edges wrong \n");

        TM_ = jt.at("TM");
        edges_ = jt.at("edges"); 
        assert(std::is_sorted(edges_.cbegin(), edges_.cend()));
      }

      TM operator()(float x) const
      {
        const auto b = std::distance(edges_.cbegin(), std::lower_bound(edges_.cbegin(), edges_.cend(), x));
        if (b == 0) return TM_[0];
        if (b >= I) return TM_[I - 1];
        const auto a = b - 1;
        const float mix = (x - edges_[a]) / (edges_[b] - edges_[a]);
        const auto& A = TM_[a];
        const auto& B = TM_[b];
        TM Y;
        for (size_t i = 0; i < Y.size(); ++i) {
          for (size_t j = 0; j < Y.size(); ++j) {
            Y[i][j] = A[i][j] + mix * (B[i][j] - A[i][j]);
          }
        }
        return Y;
      }

    private:
      std::array<TM, I> TM_;
      std::array<float, I> edges_;
    };

  }
}

#endif
