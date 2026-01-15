/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once

#include <Eigen/Eigen>

// Forward declaration - don't redefine existing types to avoid conflicts
namespace geometry_utils {
    struct Trajectory;
    // Don't redefine Mat3Df, StatePVA, StatePVAJ as they're already defined in super_utils
}

// Include the actual implementation from traj_opt namespace using relative path
#include "../../traj_opt/minco.h"

// Create namespace aliasing to map traj_opt implementations to optimization_utils interface
namespace optimization_utils {
    using namespace geometry_utils;
    using MINCO_S2NU = traj_opt::MINCO_S2NU;
    using MINCO_S3NU = traj_opt::MINCO_S3NU;
    using MINCO_S4NU = traj_opt::MINCO_S4NU;
}