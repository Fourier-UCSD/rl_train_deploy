/*
 * This file is part of broccoli.
 * Copyright (C) 2019 Chair of Applied Mechanics, Technical University of Munich
 * https://www.amm.mw.tum.de/
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../splines/QuaternionNLERPSpline.hpp"
#include "QuaternionTrajectory.hpp"

namespace broccoli {
namespace curve {
    //! Class representing a quaternion trajectory with underlying quaternion NLERP spline
    /*!
     * \ingroup broccoli_curve_trajectories
     *
     * See \ref QuaternionTrajectory and \ref QuaternionNLERPSpline for details.
     *
     * \tparam ParameterSplineType Type of parameter spline passed to \ref QuaternionTrajectory()
     */
    template <class ParameterSplineType>
    class QuaternionNLERPTrajectory : public QuaternionTrajectory<QuaternionNLERPSpline, ParameterSplineType> {
    public:
        //! Constructor
        QuaternionNLERPTrajectory()
        {
        }

        //! Destructor
        virtual ~QuaternionNLERPTrajectory()
        {
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };
} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
