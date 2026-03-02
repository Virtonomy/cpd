#pragma once
#include <vector>

struct RigidResult {
  std::vector<double> R;   // 9
  std::vector<double> t;   // 3
  std::vector<double> TY;  // M*3
};

RigidResult rigid_register(
  const std::vector<double>& Xflat, int n,
  const std::vector<double>& Yflat, int m,
  int max_iterations,
  double tolerance,
  double w_outlier,
  double sigma2 = 1.0,
  bool scale = false
);
