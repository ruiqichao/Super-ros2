// This file provides namespace aliasing to resolve the discrepancy between
// traj_opt and optimization_utils namespaces for MINCO classes
#pragma once

#include "../../traj_opt/minco.h"

// Create namespace alias to map traj_opt implementations to optimization_utils interface
namespace optimization_utils {
    using MINCO_S2NU = traj_opt::MINCO_S2NU;
    using MINCO_S3NU = traj_opt::MINCO_S3NU;
    using MINCO_S4NU = traj_opt::MINCO_S4NU;
}