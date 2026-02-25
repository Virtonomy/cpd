#include <iostream>
#include <vector>
#include "cpd_api.hpp"

int main() {
  // 4 points in 3D
  std::vector<double> X = {
    0,0,0,
    1,0,0,
    0,1,0,
    0,0,1
  };

  // the same points, but shifted by (1,1,0)
  std::vector<double> Y = {
    1,1,0,
    2,1,0,
    1,2,0,
    1,1,1
  };

  auto res = rigid_register(
    X, 4,
    Y, 4,
    100,     // iterations
    1e-6,    // tolerance
    0.0      // outliers
  );

  std::cout << "R:\n";
  for (double v : res.R) std::cout << v << " ";
  std::cout << "\n\n";

  std::cout << "t:\n";
  for (double v : res.t) std::cout << v << " ";
  std::cout << "\n\n";

  std::cout << "First TY point:\n";
  std::cout << res.TY[0] << " " << res.TY[1] << " " << res.TY[2] << "\n";
}
