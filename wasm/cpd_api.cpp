#include <vector>
#include <stdexcept>
#include <Eigen/Dense>
#include <emscripten/bind.h>
#include "cpd_api.hpp"
#include <cpd/rigid.hpp>


static Eigen::MatrixXd toMat3(const std::vector<double>& flat, int rows) {
  if (rows < 0) {
    throw std::runtime_error("Rows cannot be negative");
  }
  if (flat.size() != static_cast<std::size_t>(rows) * 3) {
    throw std::runtime_error("Expected rows*3 elements");
  }
  Eigen::MatrixXd M(rows, 3);
  for (int i = 0; i < rows; i++) {
    M(i,0) = flat[i*3 + 0];
    M(i,1) = flat[i*3 + 1];
    M(i,2) = flat[i*3 + 2];
  }
  return M;
}

RigidResult rigid_register(
  const std::vector<double>& Xflat, int n,
  const std::vector<double>& Yflat, int m,
  int max_iterations,
  double tolerance,
  double w_outlier,
  double sigma2,
  bool scale
) {
  Eigen::MatrixXd X = toMat3(Xflat, n); // target (fixed)
  Eigen::MatrixXd Y = toMat3(Yflat, m); // source (moving)

  cpd::Rigid rigid;
  rigid.max_iterations(max_iterations);
  rigid.tolerance(tolerance);
  rigid.outliers(w_outlier);
  rigid.scale(scale);
  rigid.sigma2(sigma2);

  auto res = rigid.run(X, Y);

  RigidResult out;
  out.R.resize(9);
  out.t.resize(3);
  out.TY.resize(m * 3);

  // rotation 3x3
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      out.R[r*3 + c] = res.rotation(r,c);

  // translation 3
  for (int i = 0; i < 3; i++)
    out.t[i] = res.translation(i);

  // transformed moving points: res.points (m x 3)
  for (int i = 0; i < m; i++) {
    out.TY[i*3 + 0] = res.points(i,0);
    out.TY[i*3 + 1] = res.points(i,1);
    out.TY[i*3 + 2] = res.points(i,2);
  }

  return out;
}

EMSCRIPTEN_BINDINGS(cpd_module) {
  emscripten::register_vector<double>("VectorDouble");

  emscripten::value_object<RigidResult>("RigidResult")
    .field("R", &RigidResult::R)
    .field("t", &RigidResult::t)
    .field("TY", &RigidResult::TY);

  emscripten::function("rigid_register", &rigid_register);
}
