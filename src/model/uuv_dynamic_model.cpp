#include "uuv_dynamic_model.h"

UUVDynamicModel::UUVDynamicModel(Eigen::VectorXf eta) {
  state.eta_dot = Eigen::VectorXf::Zero(6);
  state.eta_dot_prev = Eigen::VectorXf::Zero(6);
  state.nu = Eigen::VectorXf::Zero(6);
  state.nu_dot = Eigen::VectorXf::Zero(6);
  state.nu_dot_prev = Eigen::VectorXf::Zero(6);
  UUVDynamicModel();
  state.eta = eta;
  q_ << 1, 0, 0, 0;
  q_dot_prev_ = Eigen::Vector4f::Zero();
}

UUVDynamicModel::UUVDynamicModel() {
  std::cout << "init" << std::endl;
  state.eta = Eigen::VectorXf::Zero(6);
  state.eta_dot = Eigen::VectorXf::Zero(6);
  state.eta_dot_prev = Eigen::VectorXf::Zero(6);
  state.nu = Eigen::VectorXf::Zero(6);
  state.nu_dot = Eigen::VectorXf::Zero(6);
  state.nu_dot_prev = Eigen::VectorXf::Zero(6);

  J_ = Eigen::MatrixXf::Zero(6, 6);
  R_ = Eigen::Matrix3f::Identity();
  q_ << 1, 0, 0, 0;
  q_dot_prev_ = Eigen::Vector4f::Zero();

  f_x_ = Eigen::MatrixXf::Zero(6, 1);
  g_x_ = Eigen::MatrixXf::Zero(6, 6);

  M_ = Eigen::MatrixXf::Identity(6, 6);
  Eigen::VectorXf m_diag;
  m_diag = Eigen::VectorXf::Zero(6);
  m_diag << m_ + X_u_dot_, m_ + Y_v_dot_, m_ + Z_w_dot_, Ixx_ + K_p_dot_,
      Iyy_ + M_q_dot_, Izz_ + N_r_dot_;
  M_ = m_diag.asDiagonal();

  C_ = Eigen::MatrixXf::Zero(6, 6);
  D_ = Eigen::MatrixXf::Zero(6, 6);
  G_ = Eigen::MatrixXf::Zero(6, 1);
}

/*
  M_ <<
    m_-X_u_dot_, 0, 0, 0, 0, 0,
    0, m_-Y_v_dot_, 0, 0, 0, 0,
    0, 0, m_-Z_w_dot_, 0, 0, 0,
    0, 0, 0, Ixx_-K_p_dot_, 0, 0,
    0, 0, 0, 0, Iyy_-M_q_dot_, 0,
    0, 0, 0, 0, 0, Izz_-N_r_dot_;
*/

Eigen::VectorXf UUVDynamicModel::aggregate(std::array<double, 6> t) {
  Eigen::VectorXf azimuth(6), elevation(6), thrust(6);
  azimuth << -0.3491, 0.3491, 0.3491, -0.3491, 0.0, 0.0;
  elevation << 0.2618, 0.2618, -0.2618, -0.2618, 1.5708, 1.5708;
  thrust << t[0], t[1], t[2], t[3], t[4], t[5];

  Eigen::MatrixXf pos(6, 3), dir(6, 3);
  pos << 0.201, 0.19, 0.051, //
      0.201, -0.19, 0.051,   //
      -0.268, 0.195, 0.054,  //
      -0.268, -0.195, 0.054, //
      -0.039, 0.193, 0.058,  //
      -0.039, -0.193, 0.058; //
  dir.col(0) = elevation.array().cos() * azimuth.array().cos();
  dir.col(1) = elevation.array().cos() * azimuth.array().sin();
  dir.col(2) = -elevation.array().sin();

  // Thrust Allocation Matrix
  Eigen::MatrixXf tam(6, 6);
  tam.row(0) = dir.col(0).transpose();
  tam.row(1) = dir.col(1).transpose();
  tam.row(2) = dir.col(2).transpose();
  tam.row(3) = pos.col(1).array() * dir.col(2).array() -
               pos.col(2).array() * dir.col(1).array(); // y·ez - z·ey
  tam.row(4) = pos.col(2).array() * dir.col(0).array() -
               pos.col(0).array() * dir.col(2).array(); // z·ex - x·ez
  tam.row(5) = pos.col(0).array() * dir.col(1).array() -
               pos.col(1).array() * dir.col(0).array(); // x·ey - y·ex

  return tam * thrust;
}

void UUVDynamicModel::update(std::array<double, 6> t) {
  /* Input forces vector */
  Eigen::VectorXf u_ = aggregate(t);
  matricesUpdate();
  g_x_ = M_.inverse();
  // f_x_ = -M_.inverse() * (C_ * state.nu + D_ * state.nu + G_);
  f_x_ = -M_.inverse() * (C_ * state.nu + D_ * state.nu);

  state.nu_dot = f_x_ + g_x_ * (u_);
  state.nu = dt * (state.nu_dot + state.nu_dot_prev) / 2 + state.nu;
  state.nu_dot_prev = state.nu_dot;

  // Position integration (trapezoidal)
  Eigen::Vector3f pos_dot = R_ * state.nu.head(3);
  state.eta.head(3) += dt * (pos_dot + state.eta_dot.head(3)) / 2.0f;
  state.eta_dot.head(3) = pos_dot;

  // Quaternion integration — avoids the tan(theta) singularity at pitch ±90°
  Eigen::Vector3f omega;
  omega << state.nu(3), state.nu(4), state.nu(5);
  float qw = q_(0), qx = q_(1), qy = q_(2), qz = q_(3);
  Eigen::Matrix<float, 4, 3> Xi;
  Xi << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;
  Eigen::Vector4f q_dot = 0.5f * Xi * omega;
  q_ += dt * (q_dot + q_dot_prev_) / 2.0f;
  q_dot_prev_ = q_dot;
  q_.normalize();

  // Extract Euler angles from quaternion for state.eta (output only)
  qw = q_(0);
  qx = q_(1);
  qy = q_(2);
  qz = q_(3);
  state.eta(3) = std::atan2(2 * (qw * qx + qy * qz),
                            qw * qw - qx * qx - qy * qy + qz * qz);
  float sinp = std::clamp(2 * (qw * qy - qz * qx), -1.0f, 1.0f);
  state.eta(4) = std::asin(sinp);
  state.eta(5) = std::atan2(2 * (qw * qz + qx * qy),
                            qw * qw + qx * qx - qy * qy - qz * qz);

  // Printing for debug
  Eigen::IOFormat fmt(6, 0, ", ", "\n", "[", "]");
  std::cout << "force:\n" << u_.format(fmt) << "\n" << std::endl;
}

double UUVDynamicModel::constrainAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
}

void UUVDynamicModel::matricesUpdate() {
  double u, v, w, p, q, r, phi, theta, psi;
  u = state.nu(0);
  v = state.nu(1);
  w = state.nu(2);
  p = state.nu(3);
  q = state.nu(4);
  r = state.nu(5);
  phi = state.eta(3);
  theta = state.eta(4);
  psi = state.eta(5);

  // R_ from quaternion — singularity-free
  float qw = q_(0), qx = q_(1), qy = q_(2), qz = q_(3);
  R_ << 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz),
      2 * (qx * qz + qw * qy), 2 * (qx * qy + qw * qz),
      1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx),
      2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx),
      1 - 2 * (qx * qx + qy * qy);

  J_ = Eigen::MatrixXf::Zero(6, 6);
  J_.topLeftCorner(3, 3) = R_;

  double a1, a2, a3, b1, b2, b3;
  a1 = (m_ + X_u_dot_) * u;
  a2 = (m_ + Y_v_dot_) * v;
  a3 = (m_ + Z_w_dot_) * w;
  b1 = (Ixx_ + K_p_dot_) * p;
  b2 = (Iyy_ + M_q_dot_) * q;
  b3 = (Izz_ + N_r_dot_) * r;
  C_ << 0, 0, 0, 0, a3, -a2, 0, 0, 0, -a3, 0, a1, 0, 0, 0, a2, -a1, 0, 0, a3,
      -a2, 0, -b3, b2, -a3, 0, a1, b3, 0, -b1, a2, -a1, 0, -b2, b1, 0;

  D_ = Eigen::MatrixXf::Identity(6, 6);
  Eigen::VectorXf d_diag;
  d_diag = Eigen::VectorXf::Zero(6);
  d_diag << X_u_ + X_uu_ * std::fabs(u), Y_v_ + Y_vv_ * std::fabs(v),
      Z_w_ + Z_ww_ * std::fabs(w), K_p_ + K_pp_ * std::fabs(p),
      M_q_ + M_qq_ * std::fabs(q), N_r_ + N_rr_ * std::fabs(r);
  D_ = d_diag.asDiagonal() * -1;

  G_ << (W_ - B_) * std::sin(theta),
      -(W_ - B_) * std::cos(theta) * std::sin(phi),
      -(W_ - B_) * std::cos(theta) * std::cos(phi),
      -rb_z_ * B_ * std::cos(theta) * std::sin(phi),
      -rb_z_ * B_ * std::sin(theta), 0;
}
