// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "open3d/pipelines/registration/RobustKernel.h"

namespace open3d {

namespace geometry {
class PointCloud;
}

namespace pipelines {
namespace registration {

typedef std::vector<Eigen::Vector2i> CorrespondenceSet;

enum class TransformationEstimationType {
    Unspecified = 0,
    PointToPoint = 1,
    PointToPlane = 2,
    ColoredICP = 3,
    GeneralizedICP = 4,
};

/// \class TransformationEstimation
///
/// Base class that estimates a transformation between two point clouds
/// The virtual function ComputeTransformation() must be implemented in
/// subclasses.
class TransformationEstimation {
public:
    /// \brief Default Constructor.
    TransformationEstimation() {}
    virtual ~TransformationEstimation() {}

public:
    virtual TransformationEstimationType GetTransformationEstimationType()
            const = 0;
    /// Compute RMSE between source and target points cloud given
    /// correspondences.
    ///
    /// \param source Source point cloud.
    /// \param target Target point cloud.
    /// \param corres Correspondence set between source and target point cloud.
    virtual double ComputeRMSE(const geometry::PointCloud &source,
                               const geometry::PointCloud &target,
                               const CorrespondenceSet &corres) const = 0;
    /// Compute transformation from source to target point cloud given
    /// correspondences.
    ///
    /// \param source Source point cloud.
    /// \param target Target point cloud.
    /// \param corres Correspondence set between source and target point cloud.
    virtual Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            const CorrespondenceSet &corres) const = 0;
};

/// \class TransformationEstimationPointToPoint
///
/// Estimate a transformation for point to point distance.
class TransformationEstimationPointToPoint : public TransformationEstimation {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param with_scaling Set to True to estimate scaling, False to force
    /// scaling to be 1.
    TransformationEstimationPointToPoint(bool with_scaling = false)
        : with_scaling_(with_scaling) {}
    virtual ~TransformationEstimationPointToPoint() override {}

public:
    TransformationEstimationType GetTransformationEstimationType()
            const override {
        return type_;
    };
    virtual double ComputeRMSE(const geometry::PointCloud &source,
                       const geometry::PointCloud &target,
                       const CorrespondenceSet &corres) const override;
    virtual Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            const CorrespondenceSet &corres) const override;

public:
    /// \brief Set to True to estimate scaling, False to force scaling to be 1.
    ///
    /// The homogeneous transformation is given by\n
    /// T = [ cR t]\n
    ///    [0   1]\n
    /// Sets 𝑐=1 if with_scaling is False.
    bool with_scaling_ = false;

private:
    const TransformationEstimationType type_ =
            TransformationEstimationType::PointToPoint;
};

class TransformationEstimationPointToPointWithCallback: public TransformationEstimationPointToPoint {
public:
    using CallbackType = std::function<void(Eigen::Matrix4d)>;
    /// \brief Parameterized Constructor.
    ///
    /// \param with_scaling Set to True to estimate scaling, False to force
    /// scaling to be 1.
    TransformationEstimationPointToPointWithCallback(bool with_scaling, CallbackType callback_after_iteration)
        : TransformationEstimationPointToPoint(with_scaling), callback_after_iteration_(callback_after_iteration) {}
    virtual ~TransformationEstimationPointToPointWithCallback() override {}

public:
    Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            const CorrespondenceSet &corres) const override;

private:
    /// \brief Callback function which is called after each iteration.
    CallbackType callback_after_iteration_;
};

/// \class TransformationEstimationPointToPlane
///
/// Class to estimate a transformation for point to plane distance.
class TransformationEstimationPointToPlane : public TransformationEstimation {
public:
    /// \brief Default Constructor.
    TransformationEstimationPointToPlane() {}
    virtual ~TransformationEstimationPointToPlane() override {}

    /// \brief Constructor that takes as input a RobustKernel \param kernel Any
    /// of the implemented statistical robust kernel for outlier rejection.
    explicit TransformationEstimationPointToPlane(
            std::shared_ptr<RobustKernel> kernel)
        : kernel_(std::move(kernel)) {}

public:
    TransformationEstimationType GetTransformationEstimationType()
            const override {
        return type_;
    };
    double ComputeRMSE(const geometry::PointCloud &source,
                       const geometry::PointCloud &target,
                       const CorrespondenceSet &corres) const override;
    virtual Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            const CorrespondenceSet &corres) const override;

public:
    /// shared_ptr to an Abstract RobustKernel that could mutate at runtime.
    std::shared_ptr<RobustKernel> kernel_ = std::make_shared<L2Loss>();

private:
    const TransformationEstimationType type_ =
            TransformationEstimationType::PointToPlane;
};

/// \class TransformationEstimationPointToPlaneWithCallback
///
/// Class to estimate a transformation for point to plane distance.
class TransformationEstimationPointToPlaneWithCallback : public TransformationEstimationPointToPlane {
public:
    using CallbackType = std::function<void(Eigen::Matrix4d)>;
    /// \brief Default Constructor.
    TransformationEstimationPointToPlaneWithCallback(CallbackType callback_after_iteration ): callback_after_iteration_(callback_after_iteration) {}
    virtual ~TransformationEstimationPointToPlaneWithCallback() override {}

    /// \brief Constructor that takes as input a RobustKernel \param kernel Any
    /// of the implemented statistical robust kernel for outlier rejection.
    explicit TransformationEstimationPointToPlaneWithCallback(
            std::shared_ptr<RobustKernel> kernel, CallbackType callback_after_iteration )
        : TransformationEstimationPointToPlane(kernel),callback_after_iteration_(callback_after_iteration) {}

public:
    Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &source,
            const geometry::PointCloud &target,
            const CorrespondenceSet &corres) const override;
private:
    /// \brief Callback function which is called after each iteration.
    CallbackType callback_after_iteration_;
};


}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
