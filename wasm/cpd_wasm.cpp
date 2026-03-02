#include <emscripten/bind.h>
#include "cpd_api.hpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(cpd_module) {
  value_object<RigidResult>("RigidResult")
    .field("R", &RigidResult::R)
    .field("t", &RigidResult::t)
    .field("TY", &RigidResult::TY);

  register_vector<double>("VectorDouble");

  function("rigid_register", &rigid_register);
}
