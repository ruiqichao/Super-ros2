#pragma once

#include <utils/optimization/banded_system.h>
#include <data_structure/base/trajectory.h>
#include <utils/geometry/geometry_utils.h>

namespace traj_opt {
    using namespace geometry_utils;

    // MINCO for s=2 and non-uniform time
    class MINCO_S2NU {
    public:
        MINCO_S2NU() = default;

        ~MINCO_S2NU() { A.destroy(); }

    private:
        int N{0};
        Eigen::Matrix<double, 3, 2> headPV;
        Eigen::Matrix<double, 3, 2> tailPV;
        BandedSystem A;
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;

    public:
        void setConditions(const Eigen::Matrix<double, 3, 2> &headState,
                           const Eigen::Matrix<double, 3, 2> &tailState,
                           const int &pieceNum);

        void setParameters(const Eigen::Matrix3Xd &inPs,
                           const Eigen::VectorXd &ts);

        void getTrajectory(Trajectory &traj) const;

        void getEnergy(double &energy) const;

        const Eigen::MatrixX3d &getCoeffs() const;

        void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const;

        void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const;

        void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                           const Eigen::VectorXd &partialGradByTimes,
                           Eigen::Matrix3Xd &gradByPoints,
                           Eigen::VectorXd &gradByTimes);
    };

    // MINCO for s=3 and non-uniform time
    class MINCO_S3NU {
    public:
        MINCO_S3NU() = default;

        ~MINCO_S3NU() { A.destroy(); }

    private:
        int N{0};
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        BandedSystem A;
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;

    public:
        void setConditions(const Eigen::Matrix3d &headState,
                           const Eigen::Matrix3d &tailState,
                           const int &pieceNum);

        void setConditions(const Eigen::Matrix3d &headState,
                           const Eigen::Matrix3d &tailState);

        void setEndPosition(const Eigen::Vector3d &end_p);

        void setParameters(const Eigen::Matrix3Xd &inPs,
                           const Eigen::VectorXd &ts);

        void getTrajectory(Trajectory &traj) const;

        void getEnergy(double &energy) const;

        const Eigen::MatrixX3d &getCoeffs() const;

        void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const;

        void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const;

        void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                           const Eigen::VectorXd &partialGradByTimes,
                           Eigen::Matrix3Xd &gradByPoints,
                           Eigen::VectorXd &gradByTimes,
                           bool free_end = false);

        void propagateGradOfWayptsAndState(const Eigen::MatrixX3d &partialGradByCoeffs,
                                           const Eigen::VectorXd &partialGradByTimes,
                                           Eigen::VectorXd &gradByTimes,
                                           StatePVA &gradByHeadState,
                                           Mat3Df &gradByPoints,
                                           StatePVA &gradByTailState);
    };

    // MINCO for s=4 and non-uniform time
    class MINCO_S4NU {
    public:
        MINCO_S4NU() = default;

        ~MINCO_S4NU() { A.destroy(); }

    private:
        int N{0};
        Eigen::Matrix<double, 3, 4> headPVAJ;
        Eigen::Matrix<double, 3, 4> tailPVAJ;
        BandedSystem A;
        Eigen::MatrixX3d b;
        Eigen::VectorXd T1;
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
        Eigen::VectorXd T6;
        Eigen::VectorXd T7;

    public:
        void setConditions(const Eigen::Matrix<double, 3, 4> &headState,
                           const Eigen::Matrix<double, 3, 4> &tailState,
                           const int &pieceNum);

        void setConditions(const Eigen::Matrix<double, 3, 4> &headState,
                           const Eigen::Matrix<double, 3, 4> &tailState);

        void setEndPosition(const Eigen::Vector3d &end_p);

        void setParameters(const Eigen::MatrixXd &inPs,
                           const Eigen::VectorXd &ts);

        void getTrajectory(Trajectory &traj) const;

        void getEnergy(double &energy) const;

        const Eigen::MatrixX3d &getCoeffs() const;

        void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d &gdC) const;

        void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const;

        void propogateGrad(const Eigen::MatrixX3d &partialGradByCoeffs,
                           const Eigen::VectorXd &partialGradByTimes,
                           Eigen::Matrix3Xd &gradByPoints,
                           Eigen::VectorXd &gradByTimes,
                           bool free_end = false);

        void propagateGradOfWayptsAndState(const Eigen::MatrixX3d &partialGradByCoeffs,
                                           const Eigen::VectorXd &partialGradByTimes,
                                           Eigen::VectorXd &gradByTimes,
                                           StatePVAJ &gradByHeadState,
                                           Eigen::Matrix3Xd &gradByPoints,
                                           StatePVAJ &gradByTailState);
    };
}

// Create namespace alias to map traj_opt implementations to optimization_utils interface
namespace optimization_utils {
    using namespace geometry_utils;
    using MINCO_S2NU = traj_opt::MINCO_S2NU;
    using MINCO_S3NU = traj_opt::MINCO_S3NU;
    using MINCO_S4NU = traj_opt::MINCO_S4NU;
}