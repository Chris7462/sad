#pragma once

#include <istream>
#include <memory>
#include <ostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <sad_common/imu_preintegration.hpp>
#include <sad_common/nav_state.hpp>

/**
 * g2o vertices and edges used by the chapter 4 GINS.
 *
 * Book common/g2o_types.h/.cc (only the classes chapter 4 uses) merged with ch4/g2o_types.h/.cc.
 * Not ported yet, used by later chapters: EdgeGNSSTransOnly, EdgeRelativeMotion, EdgeNDT.
 *
 * Differences from the book code:
 *  - jr() / jr_inv() come from the book's modified Sophus; here they are computed from upstream
 *    Sophus with Jr(phi) = Jl(-phi), Jr^-1(phi) = Jl^-1(-phi), see SO3Jr() / SO3JrInv()
 *  - EdgeInertial initializes its members in declaration order (avoids -Wreorder)
 *  - override on every virtual function, unused parameter names commented out
 */

using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;
using Mat15d = Eigen::Matrix<double, 15, 15>;

/// Right Jacobian of SO(3)
inline Eigen::Matrix3d SO3Jr(const Eigen::Vector3d & phi)
{
  return Sophus::SO3d::leftJacobian(-phi);
}

/// Inverse of the right Jacobian of SO(3)
inline Eigen::Matrix3d SO3JrInv(const Eigen::Vector3d & phi)
{
  return Sophus::SO3d::leftJacobianInverse(-phi);
}

/// Inverse of the right Jacobian evaluated at Log(R)
inline Eigen::Matrix3d SO3JrInv(const Sophus::SO3d & R)
{
  return SO3JrInv(R.log());
}

/**
 * SO3 + t pose, rotation first, 6 DoF. Written as g2o::VertexSE3 so g2o_viewer can show it.
 */
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexPose() {}

  bool read(std::istream & is) override
  {
    double data[7];
    for (int i = 0; i < 7; i++) {
      is >> data[i];
    }
    setEstimate(
      Sophus::SE3d(
        Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
        Eigen::Vector3d(data[0], data[1], data[2])));
    return true;
  }

  bool write(std::ostream & os) const override
  {
    os << "VERTEX_SE3:QUAT ";
    os << id() << " ";
    Eigen::Quaterniond q = _estimate.unit_quaternion();
    os << _estimate.translation().transpose() << " ";
    os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] <<
      std::endl;
    return true;
  }

  void setToOriginImpl() override {}

  void oplusImpl(const double * update_) override
  {
    // rotation part
    _estimate.so3() = _estimate.so3() * Sophus::SO3d::exp(Eigen::Map<const Eigen::Vector3d>(&update_[0]));
    // translation part
    _estimate.translation() += Eigen::Map<const Eigen::Vector3d>(&update_[3]);
    updateCache();
  }
};

/**
 * Velocity vertex, a plain Vec3d
 */
class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexVelocity() {}

  bool read(std::istream & /*is*/) override {return false;}
  bool write(std::ostream & /*os*/) const override {return false;}

  void setToOriginImpl() override {_estimate.setZero();}

  void oplusImpl(const double * update_) override
  {
    _estimate += Eigen::Map<const Eigen::Vector3d>(update_);
  }
};

/**
 * Gyro bias vertex, also a Vec3d, derived from the velocity vertex
 */
class VertexGyroBias : public VertexVelocity
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexGyroBias() {}
};

/**
 * Accelerometer bias vertex, also a Vec3d, derived from the velocity vertex
 */
class VertexAccBias : public VertexVelocity
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexAccBias() {}
};

/**
 * Gyro bias random walk
 */
class EdgeGyroRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeGyroRW() {}

  bool read(std::istream & /*is*/) override {return false;}
  bool write(std::ostream & /*os*/) const override {return false;}

  void computeError() override
  {
    const auto * VG1 = dynamic_cast<const VertexGyroBias *>(_vertices[0]);
    const auto * VG2 = dynamic_cast<const VertexGyroBias *>(_vertices[1]);
    _error = VG2->estimate() - VG1->estimate();
  }

  void linearizeOplus() override
  {
    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  Eigen::Matrix<double, 6, 6> GetHessian()
  {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
};

/**
 * Accelerometer bias random walk
 */
class EdgeAccRW : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeAccRW() {}

  bool read(std::istream & /*is*/) override {return false;}
  bool write(std::ostream & /*os*/) const override {return false;}

  void computeError() override
  {
    const auto * VA1 = dynamic_cast<const VertexAccBias *>(_vertices[0]);
    const auto * VA2 = dynamic_cast<const VertexAccBias *>(_vertices[1]);
    _error = VA2->estimate() - VA1->estimate();
  }

  void linearizeOplus() override
  {
    _jacobianOplusXi = -Eigen::Matrix3d::Identity();
    _jacobianOplusXj.setIdentity();
  }

  Eigen::Matrix<double, 6, 6> GetHessian()
  {
    linearizeOplus();
    Eigen::Matrix<double, 3, 6> J;
    J.block<3, 3>(0, 0) = _jacobianOplusXi;
    J.block<3, 3>(0, 3) = _jacobianOplusXj;
    return J.transpose() * information() * J;
  }
};

/**
 * Prior on the previous IMU state (pvq + biases).
 * The information matrix is given from outside, from the marginalization of the time window.
 *
 * Vertex order: pose, v, bg, ba
 * Residual order: R, p, v, bg, ba, 15 dimensions
 */
class EdgePriorPoseNavState : public g2o::BaseMultiEdge<15, Vec15d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePriorPoseNavState(const NavState & state, const Mat15d & info);

  bool read(std::istream & /*is*/) override {return false;}
  bool write(std::ostream & /*os*/) const override {return false;}

  void computeError() override;
  void linearizeOplus() override;

  Eigen::Matrix<double, 15, 15> GetHessian()
  {
    linearizeOplus();
    Eigen::Matrix<double, 15, 15> J;
    J.block<15, 6>(0, 0) = _jacobianOplus[0];
    J.block<15, 3>(0, 6) = _jacobianOplus[1];
    J.block<15, 3>(0, 9) = _jacobianOplus[2];
    J.block<15, 3>(0, 12) = _jacobianOplus[3];
    return J.transpose() * information() * J;
  }

  NavState state_;
};

/**
 * 6-DoF GNSS. The residual has the rotation first, then the translation.
 */
class EdgeGNSS : public g2o::BaseUnaryEdge<6, Sophus::SE3d, VertexPose>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeGNSS() = default;
  EdgeGNSS(VertexPose * v, const Sophus::SE3d & obs)
  {
    setVertex(0, v);
    setMeasurement(obs);
  }

  void computeError() override
  {
    VertexPose * v = (VertexPose *)_vertices[0];
    _error.head<3>() = (_measurement.so3().inverse() * v->estimate().so3()).log();
    _error.tail<3>() = v->estimate().translation() - _measurement.translation();
  }

  void linearizeOplus() override
  {
    VertexPose * v = (VertexPose *)_vertices[0];
    // 6x6 Jacobian
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<3, 3>(0, 0) =
      SO3JrInv(_measurement.so3().inverse() * v->estimate().so3());   // dR/dR
    _jacobianOplusXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // dp/dp
  }

  Eigen::Matrix<double, 6, 6> GetHessian()
  {
    linearizeOplus();
    return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
  }

  bool read(std::istream & /*in*/) override {return true;}
  bool write(std::ostream & /*out*/) const override {return true;}
};

/**
 * 3D wheel speed observation.
 * Observes the world-frame velocity; in 3D the vehicle is assumed to have no y or z velocity.
 */
class EdgeEncoder3D : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexVelocity>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeEncoder3D() = default;

  /**
   * @param v0     velocity vertex
   * @param speed  velocity in the world frame
   */
  EdgeEncoder3D(VertexVelocity * v0, const Eigen::Vector3d & speed)
  {
    setVertex(0, v0);
    setMeasurement(speed);
  }

  void computeError() override
  {
    VertexVelocity * v0 = (VertexVelocity *)_vertices[0];
    _error = v0->estimate() - _measurement;
  }

  void linearizeOplus() override {_jacobianOplusXi.setIdentity();}

  bool read(std::istream & /*in*/) override {return true;}
  bool write(std::ostream & /*out*/) const override {return true;}
};

/**
 * Preintegration edge.
 * Connects 6 vertices: pose, v, bg, ba of the previous frame and pose, v of the current frame.
 * The measurement is the 9-dimensional preintegration residual, order: R, v, p.
 * The information matrix comes from the preintegration and is computed in the constructor.
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vec9d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @param preinteg  preintegration object
   * @param gravity   gravity vector
   * @param weight    weight
   */
  EdgeInertial(
    std::shared_ptr<IMUPreintegration> preinteg, const Eigen::Vector3d & gravity,
    double weight = 1.0);

  bool read(std::istream & /*is*/) override {return false;}
  bool write(std::ostream & /*os*/) const override {return false;}

  void computeError() override;
  void linearizeOplus() override;

  Eigen::Matrix<double, 24, 24> GetHessian()
  {
    linearizeOplus();
    Eigen::Matrix<double, 9, 24> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[0];
    J.block<9, 3>(0, 6) = _jacobianOplus[1];
    J.block<9, 3>(0, 9) = _jacobianOplus[2];
    J.block<9, 3>(0, 12) = _jacobianOplus[3];
    J.block<9, 6>(0, 15) = _jacobianOplus[4];
    J.block<9, 3>(0, 21) = _jacobianOplus[5];
    return J.transpose() * information() * J;
  }

private:
  const double dt_;
  std::shared_ptr<IMUPreintegration> preint_ = nullptr;
  Eigen::Vector3d grav_;
};
