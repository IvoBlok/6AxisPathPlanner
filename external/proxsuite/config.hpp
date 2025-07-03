#pragma once

// ———————————————————— Version Info ————————————————————
// (You can adjust these to match your branch version if needed)
#define PROXSUITE_VERSION_MAJOR 0
#define PROXSUITE_VERSION_MINOR 7
#define PROXSUITE_VERSION_PATCH 2
#define PROXSUITE_VERSION "0.7.2"

// —————————————————— Configuration Flags ——————————————————

#define PROXSUITE_WITH_EIGEN 1          // We've included Eigen headers
#define PROXSUITE_USE_DOUBLE_PRECISION 1
#define PROXSUITE_DEBUG_MODE \
  (defined(NDEBUG) ? 0 : 1)           // 0 = Release, 1 = Debug
#define PROXSUITE_WITH_CRT_STATIC 0     // Assume dynamic CRT (change if static)