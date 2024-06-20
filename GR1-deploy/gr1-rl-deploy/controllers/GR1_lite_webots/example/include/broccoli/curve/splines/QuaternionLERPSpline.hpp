/*
 * This file is part of broccoli
 * Copyright (C) 2021 Chair of Applied Mechanics, Technical University of Munich
 * https://www.mw.tum.de/am
 */

#pragma once // Load this file only once

// This module requires Eigen library
#ifdef HAVE_EIGEN3

#include "../curves/QuaternionLERPCurve.hpp"
#include "InterpolatableQuaternionSpline.hpp"
#include <Eigen/StdVector>

namespace broccoli {
namespace curve {
    //! Class representing a quaternion spline as concatenation of quaternion LERP curve segments
    /*!
     * \ingroup broccoli_curve_splines
     *
     * See \ref InterpolatableQuaternionSpline and \ref QuaternionLERPCurve for details.
     */
    class QuaternionLERPSpline : public QuaternionSpline<QuaternionLERPCurve>, public InterpolatableQuaternionSpline {
    public:
        //! Constructor
        QuaternionLERPSpline()
        {
        }

        //! Destructor
        virtual ~QuaternionLERPSpline()
        {
        }

        // Interpolation
        // -------------
        // Interpolate parameters of underlying curve to fit specific constraints (see base class for details)
        //! \copydoc InterpolatableQuaternionSpline::interpolate()
        /*!
         * Notes on antipodal keyframes
         * ----------------------------
         * Note that \f$q\f$ and \f$-q\f$ describe the same rotation/orientation. However, using one or the other may change the result of interpolation which often tries to "walk the shortest path". In order to
         * avoid unexpected motion during interpolation, the method \ref QuaternionCurve::recalculateKeyFrameSequence() allows to automatically flip quaternions in a sequence of keyframes such that two neighboring
         * keyframes always have the shortest distance. This method is **automatically** called on \p parameters, to prevent undesired effects. Thus there is no need to take special care on the user-side.
         */
        virtual bool interpolate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, const QuaternionSplineInterpolationMethod& method, QuaternionSplineResult* const result = nullptr)
        {
            // Recompute keyframe sequence to fix issue with antipodal keyframe quaternions
            std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> keyFrames = QuaternionCurve::recalculateKeyFrameSequence(parameters);

            // Check interpolation method
            switch (method) {
            case QuaternionSplineInterpolationMethod::LERP_PIECEWISE: {
                /*! Interpolation with LERP_PIECEWISE
                 *  ---------------------------------
                 * \copydetails interpolatePiecewise()
                 */
                return interpolatePiecewise(keyFrames, proportions, result);
            }
            default: {
                // No interpolation methods implemented for now
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_NOTIMPLEMENTED;
                assert(false);
                return false;
            }
            }

            // Success
            return true;
        }

    protected:
        //! **Piecewise** interpolation with LERP segments with given control points (see \ref interpolate() for details on \p parameters, \p proportions and \p result)
        /*!
         * Interpolate with <b>L</b>inear int<b>ERP</b>olation (LERP) (see \ref QuaternionLERPCurve) between start- and end-quaternions
         * of each segment. Resulting quaternion spline is \f$ C^0\f$-continuous.
         *
         * \par Parameter-layout
         * Element         | Description
         * -------         | -----------
         * parameters[0]   | keyframe quaternion at beginning of first segment
         * ...             | ...
         * parameters[n-1] | keyframe quaternion at beginning of last segment
         * parameters[n]   | keyframe quaternion at end of last segment
         */
        virtual bool interpolatePiecewise(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>& parameters, const std::vector<double>* const proportions, QuaternionSplineResult* const result = nullptr)
        {
            // Check, if proportions are specified
            bool proportionsSpecified = false;
            if (proportions != nullptr)
                if (proportions->size() > 0)
                    proportionsSpecified = true;

            // Clear current data
            m_segments.clear();
            if (proportionsSpecified == false)
                m_segmentProportions.clear();
            else
                m_segmentProportions = *proportions;

            // Checking parameter set
            if (parameters.size() <= 1 || (proportionsSpecified == true && m_segmentProportions.size() != parameters.size() - 1)) {
                if (result != nullptr)
                    *result = QuaternionSplineResult::ERROR_INVALID_PARAMETERS;
                assert(false);
                return false;
            }

            // Initialize helpers
            const size_t segmentCount = parameters.size() - 1;

            // Allocate memory
            m_segments.resize(segmentCount);
            if (proportionsSpecified == false) // use uniform distribution
                m_segmentProportions.resize(segmentCount, (double)1.0 / segmentCount);

            // Create segments and set control-point quaternions
            for (size_t i = 0; i < segmentCount; i++) {
                m_segments[i].m_controlPoints[0] = parameters[i];
                m_segments[i].m_controlPoints[1] = parameters[i + 1];
            }

            // Success!
            if (result != nullptr)
                *result = QuaternionSplineResult::SUCCESS;
            return true;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <-- Proper 128 bit alignment of member data necessary for Eigen vectorization
    };

} // namespace curve
} // namespace broccoli

#endif // HAVE_EIGEN3
