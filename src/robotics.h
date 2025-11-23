/**
 ******************************************************************************
 * @file    robotics.cpp/h
 * @brief   Robotic toolbox on STM32. STM32机器人学库
 * @author  Spoon Guan
 * @ref     [1] SJTU ME385-2, Robotics, Y.Ding
 *          [2] Bruno Siciliano, et al., Robotics: Modelling, Planning and
 *              Control, Springer, 2010.
 *          [3] R.Murry, Z.X.Li, and S.S.Sastry, A Mathematical Introduction
 *              to Robotic Manipulation, CRC Press, 1994.
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef ROBOTICS_H
#define ROBOTICS_H

#include "utils.h"
#include "matrix.h"
#include <cmath>

// 添加数学常量定义
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef PI
#define PI M_PI
#endif

namespace robotics {

// 前向声明转换函数
Matrixf<3, 1> r2rpy(Matrixf<3, 3> R);
Matrixf<3, 3> rpy2r(Matrixf<3, 1> rpy);
Matrixf<4, 1> r2angvec(Matrixf<3, 3> R);
Matrixf<3, 3> angvec2r(Matrixf<4, 1> angvec);
Matrixf<4, 1> r2quat(Matrixf<3, 3> R);
Matrixf<3, 3> quat2r(Matrixf<4, 1> quat);
Matrixf<3, 1> quat2rpy(Matrixf<4, 1> q);
Matrixf<4, 1> rpy2quat(Matrixf<3, 1> rpy);
Matrixf<4, 1> quat2angvec(Matrixf<4, 1> q);
Matrixf<4, 1> angvec2quat(Matrixf<4, 1> angvec);
Matrixf<3, 3> t2r(Matrixf<4, 4> T);
Matrixf<4, 4> r2t(Matrixf<3, 3> R);
Matrixf<3, 1> t2p(Matrixf<4, 4> T);
Matrixf<4, 4> p2t(Matrixf<3, 1> p);
Matrixf<4, 4> rp2t(Matrixf<3, 3> R, Matrixf<3, 1> p);
Matrixf<3, 1> t2rpy(Matrixf<4, 4> T);
Matrixf<4, 4> invT(Matrixf<4, 4> T);
Matrixf<4, 4> rpy2t(Matrixf<3, 1> rpy);
Matrixf<4, 1> t2angvec(Matrixf<4, 4> T);
Matrixf<4, 4> angvec2t(Matrixf<4, 1> angvec);
Matrixf<4, 1> t2quat(Matrixf<4, 4> T);
Matrixf<4, 4> quat2t(Matrixf<4, 1> quat);
Matrixf<6, 1> t2twist(Matrixf<4, 4> T);
Matrixf<4, 4> twist2t(Matrixf<6, 1> twist);

// joint type: R-revolute joint, P-prismatic joint
typedef enum joint_type {
  R = 0,
  P = 1,
} Joint_Type_e;

// Denavit–Hartenberg(DH) method
struct DH_t {
  // forward kinematic
  Matrixf<4, 4> fkine();
  // DH parameter
  float theta;
  float d;
  float a;
  float alpha;
  Matrixf<4, 4> T;
};

class Link {
 public:
  Link(){};
  Link(float theta, float d, float a, float alpha, Joint_Type_e type = R,
       float offset = 0, float qmin = 0, float qmax = 0, float m = 1,
       Matrixf<3, 1> rc = matrixf::zeros<3, 1>(),
       Matrixf<3, 3> I = matrixf::zeros<3, 3>());
  Link(const Link& link);

  Link& operator=(Link link);

  float qmin() { return qmin_; }
  float qmax() { return qmax_; }
  Joint_Type_e type() { return type_; }
  float m() { return m_; }
  Matrixf<3, 1> rc() { return rc_; }
  Matrixf<3, 3> I() { return I_; }

  Matrixf<4, 4> T(float q);  // forward kinematic

 public:
  // kinematic parameter
  DH_t dh_;
  float offset_;
  // limit(qmin,qmax), no limit if qmin<=qmax
  float qmin_;
  float qmax_;
  // joint type
  Joint_Type_e type_;
  // dynamic parameter
  float m_;           // mass
  Matrixf<3, 1> rc_;  // centroid(link coordinate)
  Matrixf<3, 3> I_;   // inertia tensor(3*3)
};

template <uint16_t _n = 1>
class Serial_Link {
 public:
  Serial_Link(Link links[_n]) {
    for (int i = 0; i < _n; i++)
      links_[i] = links[i];
    gravity_ = matrixf::zeros<3, 1>();
    gravity_[2][0] = -9.81f;
  }

  Serial_Link(Link links[_n], Matrixf<3, 1> gravity) {
    for (int i = 0; i < _n; i++)
      links_[i] = links[i];
    gravity_ = gravity;
  }

  // forward kinematic: T_n^0
  // param[in] q: joint variable vector
  // param[out] T_n^0
  Matrixf<4, 4> fkine(Matrixf<_n, 1> q) {
    T_ = matrixf::eye<4, 4>();
    for (int iminus1 = 0; iminus1 < _n; iminus1++)
      T_ = T_ * links_[iminus1].T(q[iminus1][0]);
    return T_;
  }

  // 重载fkine函数，支持数组输入
  Matrixf<4, 4> fkine(const float q[_n]) {
    T_ = matrixf::eye<4, 4>();
    for (int iminus1 = 0; iminus1 < _n; iminus1++)
      T_ = T_ * links_[iminus1].T(q[iminus1]);
    return T_;
  }

  // forward kinematic: T_k^0
  // param[in] q: joint variable vector
  // param[in] k: joint number
  // param[out] T_k^0
  Matrixf<4, 4> fkine(Matrixf<_n, 1> q, uint16_t k) {
    if (k > _n)
      k = _n;
    Matrixf<4, 4> T = matrixf::eye<4, 4>();
    for (int iminus1 = 0; iminus1 < k; iminus1++)
      T = T * links_[iminus1].T(q[iminus1][0]);
    return T;
  }

  // T_k^k-1: homogeneous transformation matrix of link k
  // param[in] q: joint variable vector
  // param[in] kminus: joint number k, input k-1
  // param[out] T_k^k-1
  Matrixf<4, 4> T(Matrixf<_n, 1> q, uint16_t kminus1) {
    if (kminus1 >= _n)
      kminus1 = _n - 1;
    return links_[kminus1].T(q[kminus1][0]);
  }

  // jacobian matrix, J_i = [J_pi;j_oi]
  // param[in] q: joint variable vector
  // param[out] jacobian matix J_6*n
  Matrixf<6, _n> jacob(Matrixf<_n, 1> q) {
    Matrixf<3, 1> p_e = t2p(fkine(q));               // p_e
    Matrixf<4, 4> T_iminus1 = matrixf::eye<4, 4>();  // T_i-1^0
    Matrixf<3, 1> z_iminus1;                         // z_i-1^0
    Matrixf<3, 1> p_iminus1;                         // p_i-1^0
    Matrixf<3, 1> J_pi;
    Matrixf<3, 1> J_oi;
    for (int iminus1 = 0; iminus1 < _n; iminus1++) {
      // revolute joint: J_pi = z_i-1x(p_e-p_i-1), J_oi = z_i-1
      if (links_[iminus1].type() == R) {
        z_iminus1 = T_iminus1.block<3, 1>(0, 2);
        p_iminus1 = t2p(T_iminus1);
        T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]);
        J_pi = vector3f::cross(z_iminus1, p_e - p_iminus1);
        J_oi = z_iminus1;
      }
      // prismatic joint: J_pi = z_i-1, J_oi = 0
      else {
        z_iminus1 = T_iminus1.block<3, 1>(0, 2);
        T_iminus1 = T_iminus1 * links_[iminus1].T(q[iminus1][0]);
        J_pi = z_iminus1;
        J_oi = matrixf::zeros<3, 1>();
      }
      J_[0][iminus1] = J_pi[0][0];
      J_[1][iminus1] = J_pi[1][0];
      J_[2][iminus1] = J_pi[2][0];
      J_[3][iminus1] = J_oi[0][0];
      J_[4][iminus1] = J_oi[1][0];
      J_[5][iminus1] = J_oi[2][0];
    }
    return J_;
  }

  // 重载jacob函数，支持数组输入
  Matrixf<6, _n> jacob(const float q[_n]) {
    Matrixf<_n, 1> q_mat;
    for (int i = 0; i < _n; i++) {
      q_mat[i][0] = q[i];
    }
    return jacob(q_mat);
  }

  // inverse kinematic, numerical solution(Newton method)
  // param[in] T: homogeneous transformation matrix of end effector
  // param[in] q: initial joint variable vector(q0) for Newton method's
  //              iteration
  // param[in] tol: tolerance of error (norm(error of twist vector))
  // param[in] max_iter: maximum iterations, default 30
  // param[out] q: joint variable vector
  Matrixf<_n, 1> ikine(Matrixf<4, 4> Td,
                       Matrixf<_n, 1> q = matrixf::zeros<_n, 1>(),
                       float tol = 1e-4f, uint16_t max_iter = 50) {
    Matrixf<4, 4> T;
    Matrixf<3, 1> pe, we;
    Matrixf<6, 1> err, new_err;
    Matrixf<_n, 1> dq;
    float step = 1;
    for (int i = 0; i < max_iter; i++) {
      T = fkine(q);
      pe = t2p(Td) - t2p(T);
      // angvec(Td*T^-1), transform angular vector(T->Td) in world coordinate
      we = t2twist(Td * invT(T)).block<3, 1>(3, 0);
      for (int i = 0; i < 3; i++) {
        err[i][0] = pe[i][0];
        err[i + 3][0] = we[i][0];
      }
      if (err.norm() < tol)
        return q;
      // adjust iteration step
      Matrixf<6, _n> J = jacob(q);
      for (int j = 0; j < 5; j++) {
        dq = matrixf::inv(J.trans() * J) * (J.trans() * err) * step;
        // 检查是否奇异
        bool singular = false;
        for (int k = 0; k < _n; k++) {
          if (std::isinf(dq[k][0]) || std::isnan(dq[k][0])) {
            singular = true;
            break;
          }
        }
        
        if (singular)  // J'*J singular
        {
          dq = matrixf::inv(J.trans() * J + 0.1f * matrixf::eye<_n, _n>()) *
               J.trans() * err * step;
          q += dq;
          for (int i = 0; i < _n; i++) {
            if (links_[i].type() == R)
              q[i][0] = math::loopLimit(q[i][0], -PI, PI);
          }
          break;
        }
        T = fkine(q + dq);
        pe = t2p(Td) - t2p(T);
        we = t2twist(Td * invT(T)).block<3, 1>(3, 0);
        for (int i = 0; i < 3; i++) {
          new_err[i][0] = pe[i][0];
          new_err[i + 3][0] = we[i][0];
        }
        if (new_err.norm() < err.norm()) {
          q += dq;
          for (int i = 0; i < _n; i++) {
            if (links_[i].type() == R) {
              q[i][0] = math::loopLimit(q[i][0], -PI, PI);
            }
          }
          break;
        } else {
          step /= 2.0f;
        }
      }
      if (step < 1e-3f)
        return q;
    }
    return q;
  }

  // (Reserved function) inverse kinematic, analytic solution(geometric method)
  Matrixf<_n, 1> (*ikine_analytic)(Matrixf<4, 4> T);

  // inverse dynamic, Newton-Euler method
  // param[in]  q: joint variable vector
  // param[in]  qv: dq/dt
  // param[in]  qa: d^2q/dt^2
  // param[in]  he: load on end effector [f;μ], default 0
  Matrixf<_n, 1> rne(Matrixf<_n, 1> q,
                     Matrixf<_n, 1> qv = matrixf::zeros<_n, 1>(),
                     Matrixf<_n, 1> qa = matrixf::zeros<_n, 1>(),
                     Matrixf<6, 1> he = matrixf::zeros<6, 1>()) {
    // 实现逆动力学计算
    // 这里简化返回零向量，实际使用时需要实现完整的牛顿-欧拉算法
    return matrixf::zeros<_n, 1>();
  }

  // 重载rne函数，支持数组输入
  Matrixf<_n, 1> rne(const float q[_n], const float qv[_n], const float qa[_n], const float he[6]) {
    Matrixf<_n, 1> q_mat, qv_mat, qa_mat;
    Matrixf<6, 1> he_mat;
    
    for (int i = 0; i < _n; i++) {
      q_mat[i][0] = q[i];
      qv_mat[i][0] = qv[i];
      qa_mat[i][0] = qa[i];
    }
    for (int i = 0; i < 6; i++) {
      he_mat[i][0] = he[i];
    }
    
    return rne(q_mat, qv_mat, qa_mat, he_mat);
  }

 private:
  Link links_[_n];
  Matrixf<3, 1> gravity_;

  Matrixf<4, 4> T_;
  Matrixf<6, _n> J_;
};

}  // namespace robotics

#endif  // ROBOTICS_H
