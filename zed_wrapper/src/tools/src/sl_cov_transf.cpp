///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "sl_cov_transf.h"
#include "sophus/se3.hpp"

namespace sl_tools {

    static inline Eigen::Matrix3f getNegativeSkew(Eigen::Vector3f& v) {
        Eigen::Matrix3f vs;
        vs << 0.f, v[2], -v[1],
        -v[2], 0.f, v[0],
        v[1], -v[0], 0.f;

        return vs;
    }

    static inline Eigen::Matrix<float, 3, 9> JacobianSO3Log(Eigen::Matrix3f& R) {
        float b = 0.5f;
        float a1 = 0.f;
        float a2 = 0.f;
        float a3 = 0.f;

        float A = R(0, 0) + R(1, 1) + R(2, 2) - 1.f;
        float cos_teta = std::max(std::min(A * 0.5f, 1.f), -1.f);
        float teta = acos(cos_teta);
        bool test_teta_normal = cos_teta < 0.99999f;

        if (test_teta_normal) {
            const float sin_teta = sin(teta);
            const float cts = (teta * cos_teta - sin_teta) / (4.f * sin_teta * sin_teta * sin_teta);
            b = teta / (2.f * sin_teta);

            // a
            a1 = R(2, 1) - R(1, 2);
            a2 = R(0, 2) - R(2, 0);
            a3 = R(1, 0) - R(0, 1);

            a1 *= cts;
            a2 *= cts;
            a3 *= cts;
        }

        Eigen::Matrix<float, 3, 9> JL;

        JL << a1, 0, 0, 0, a1, b, 0, -b, a1,
        a2, 0, -b, 0, a2, 0, b, 0, a2,
        a3, b, 0, -b, a3, 0, 0, 0, a3;

        return JL;
    }

    static inline Eigen::Matrix<float, 6, 12> jacobianSE3Log(Eigen::Matrix4f& RT) {
        float b = 0.5f;
        float a1 = 0.f;
        float a2 = 0.f;
        float a3 = 0.f;

        float A = RT(0, 0) + RT(1, 1) + RT(2, 2) - 1.f;
        float cos_teta = std::max(std::min(A * 0.5f, 1.f), -1.f);
        float teta = acos(cos_teta);
        bool test_teta_normal = cos_teta < 0.99999f;

        if (test_teta_normal) {
            const float sin_teta = sin(teta);
            const float cts = (teta * cos_teta - sin_teta) / (4.f * sin_teta * sin_teta * sin_teta);
            b = teta / (2.f * sin_teta);

            // a
            a1 = RT(2, 1) - RT(1, 2);
            a2 = RT(0, 2) - RT(2, 0);
            a3 = RT(1, 0) - RT(0, 1);

            a1 *= cts;
            a2 *= cts;
            a3 *= cts;
        }

        Eigen::Matrix<float, 3, 9> JL;

        JL << a1, 0, 0, 0, a1, b, 0, -b, a1,
        a2, 0, -b, 0, a2, 0, b, 0, a2,
        a3, b, 0, -b, a3, 0, 0, 0, a3;


        const Eigen::Matrix3f& rotation = RT.template block<3, 3>(0, 0);
        Eigen::Vector3f w = Sophus::SO3f(rotation).log();

        Eigen::Matrix<float, 3, 6> J_LOG_T = Eigen::Matrix<float, 3, 6>::Zero();

        float& w1 = w(0);
        float& w2 = w(1);
        float& w3 = w(2);
        float& T1 = RT(0, 3);
        float& T2 = RT(1, 3);
        float& T3 = RT(2, 3);

        if (!test_teta_normal) {
            J_LOG_T(0, 0) = (w2 * w2) * (-1.0 / 1.2E1) - (w3 * w3) * (1.0 / 1.2E1) + 1.0;
            J_LOG_T(0, 1) = w3 * (1.0 / 2.0) + w1 * w2 * (1.0 / 1.2E1);
            J_LOG_T(0, 2) = w2 * (-1.0 / 2.0) + w1 * w3 * (1.0 / 1.2E1);
            J_LOG_T(0, 3) = T2 * w2 * (1.0 / 1.2E1) + T3 * w3 * (1.0 / 1.2E1);
            J_LOG_T(0, 4) = T3 * (-1.0 / 2.0) - T1 * w2 * (1.0 / 6.0) + T2 * w1 * (1.0 / 1.2E1);
            J_LOG_T(0, 5) = T2 * (1.0 / 2.0) - T1 * w3 * (1.0 / 6.0) + T3 * w1 * (1.0 / 1.2E1);
            J_LOG_T(1, 0) = w3 * (-1.0 / 2.0) + w1 * w2 * (1.0 / 1.2E1);
            J_LOG_T(1, 1) = (w1 * w1) * (-1.0 / 1.2E1) - (w3 * w3) * (1.0 / 1.2E1) + 1.0;
            J_LOG_T(1, 2) = w1 * (1.0 / 2.0) + w2 * w3 * (1.0 / 1.2E1);
            J_LOG_T(1, 3) = T3 * (1.0 / 2.0) + T1 * w2 * (1.0 / 1.2E1) - T2 * w1 * (1.0 / 6.0);
            J_LOG_T(1, 4) = T1 * w1 * (1.0 / 1.2E1) + T3 * w3 * (1.0 / 1.2E1);
            J_LOG_T(1, 5) = T1 * (-1.0 / 2.0) - T2 * w3 * (1.0 / 6.0) + T3 * w2 * (1.0 / 1.2E1);
            J_LOG_T(2, 0) = w2 * (1.0 / 2.0) + w1 * w3 * (1.0 / 1.2E1);
            J_LOG_T(2, 1) = w1 * (-1.0 / 2.0) + w2 * w3 * (1.0 / 1.2E1);
            J_LOG_T(2, 2) = (w1 * w1) * (-1.0 / 1.2E1) - (w2 * w2) * (1.0 / 1.2E1) + 1.0;
            J_LOG_T(2, 3) = T2 * (-1.0 / 2.0) + T1 * w3 * (1.0 / 1.2E1) - T3 * w1 * (1.0 / 6.0);
            J_LOG_T(2, 4) = T1 * (1.0 / 2.0) + T2 * w3 * (1.0 / 1.2E1) - T3 * w2 * (1.0 / 6.0);
            J_LOG_T(2, 5) = T1 * w1 * (1.0 / 1.2E1) + T2 * w2 * (1.0 / 1.2E1);

        } else {
            float norm_w = sqrt(w1 * w1 + w2 * w2 + w3 * w3);
            float i_tan_norm = 1.f / (2.f * tan(norm_w * 0.5f));
            float s_norm_w = sin(norm_w);
            float c_norm_w = cos(norm_w);

            J_LOG_T(0, 0) = 1.0 / (norm_w * norm_w) * (w2 * w2 + w3 * w3) * (i_tan_norm * norm_w - 1.0) + 1.0;
            J_LOG_T(0, 1) = w3 * (1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w1 * w2 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(0, 2) = w2 * (-1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w1 * w3 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(0, 3) = -T2 * (1.0 / (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) - 1.0 /
                                   (norm_w * norm_w * norm_w * norm_w) * (w1 * w1) * w2 * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                           (norm_w * norm_w * norm_w) * (w1 * w1) * w2 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T3 * (1.0 /
                                                   (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * (w1 * w1) * w3 *
                                                   (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * (w1 * w1) * w3 * (norm_w - s_norm_w) *
                                                           (1.0 / 2.0)) / (c_norm_w - 1.0)) - T1 * (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * (w2 * w2 + w3 * w3) *
                                                                   (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 / (norm_w * norm_w * norm_w) * w1 * (w2 * w2 + w3 * w3) *
                                                                           (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(0, 4) = T1 * (1.0 / (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w2 * (w2 * w2 + w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w2 * (w2 * w2 + w3 * w3) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T3 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * -2.0 + (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T2 *
                            (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 *
                             (w2 * w2) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * w1 * (w2 * w2) *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(0, 5) = T1 * (1.0 / (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w3 * (w2 * w2 + w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w3 * (w2 * w2 + w3 * w3) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) + T2 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T3 *
                            (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 *
                             (w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * w1 * (w3 * w3) *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(1, 0) = w3 * (-1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w1 * w2 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(1, 1) = 1.0 / (norm_w * norm_w) * (w1 * w1 + w3 * w3) * (i_tan_norm * norm_w - 1.0) + 1.0;
            J_LOG_T(1, 2) = w1 * (1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w2 * w3 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(1, 3) = T2 * (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w1 * (w1 * w1 + w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w1 * (w1 * w1 + w3 * w3) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) + T3 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T1 *
                            (1.0 / (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) *
                             (w1 * w1) * w2 * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * (w1 * w1) * w2 *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(1, 4) = -T1 * (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) - 1.0 /
                                   (norm_w * norm_w * norm_w * norm_w) * w1 * (w2 * w2) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                           (norm_w * norm_w * norm_w) * w1 * (w2 * w2) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T3 * (1.0 /
                                                   (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * (w2 * w2) * w3 *
                                                   (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * (w2 * w2) * w3 * (norm_w - s_norm_w) *
                                                           (1.0 / 2.0)) / (c_norm_w - 1.0)) - T2 * (1.0 / (norm_w * norm_w * norm_w * norm_w) * w2 * (w1 * w1 + w3 * w3) *
                                                                   (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 / (norm_w * norm_w * norm_w) * w2 * (w1 * w1 + w3 * w3) *
                                                                           (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(1, 5) = T2 * (1.0 / (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w3 * (w1 * w1 + w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w3 * (w1 * w1 + w3 * w3) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T1 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * -2.0 + (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T3 *
                            (1.0 / (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * w2 *
                             (w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * w2 * (w3 * w3) *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(2, 0) = w2 * (1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w1 * w3 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(2, 1) = w1 * (-1.0 / 2.0) - 1.0 / (norm_w * norm_w) * w2 * w3 * (i_tan_norm * norm_w - 1.0);
            J_LOG_T(2, 2) = 1.0 / (norm_w * norm_w) * (w1 * w1 + w2 * w2) * (i_tan_norm * norm_w - 1.0) + 1.0;
            J_LOG_T(2, 3) = T3 * (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w1 * (w1 * w1 + w2 * w2) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w1 * (w1 * w1 + w2 * w2) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T2 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * -2.0 + (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T1 *
                            (1.0 / (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) *
                             (w1 * w1) * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * (w1 * w1) * w3 *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(2, 4) = T3 * (1.0 / (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) * 2.0 - 1.0 /
                                  (norm_w * norm_w * norm_w * norm_w) * w2 * (w1 * w1 + w2 * w2) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                          (norm_w * norm_w * norm_w) * w2 * (w1 * w1 + w2 * w2) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) + T1 *
                            (1.0 / (norm_w * norm_w * norm_w * norm_w) * w1 * w2 * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 /
                                    (norm_w * norm_w * norm_w) * w1 * w2 * w3 * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0) + 1.0 / 2.0) - T2 *
                            (1.0 / (norm_w * norm_w) * w3 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) *
                             (w2 * w2) * w3 * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * (w2 * w2) * w3 *
                                     (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));
            J_LOG_T(2, 5) = -T1 * (1.0 / (norm_w * norm_w) * w1 * (i_tan_norm * norm_w - 1.0) - 1.0 /
                                   (norm_w * norm_w * norm_w * norm_w) * w1 * (w3 * w3) * (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 /
                                           (norm_w * norm_w * norm_w) * w1 * (w3 * w3) * (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0)) - T2 * (1.0 /
                                                   (norm_w * norm_w) * w2 * (i_tan_norm * norm_w - 1.0) - 1.0 / (norm_w * norm_w * norm_w * norm_w) * w2 * (w3 * w3) *
                                                   (i_tan_norm * norm_w - 1.0) * 2.0 + (1.0 / (norm_w * norm_w * norm_w) * w2 * (w3 * w3) * (norm_w - s_norm_w) *
                                                           (1.0 / 2.0)) / (c_norm_w - 1.0)) - T3 * (1.0 / (norm_w * norm_w * norm_w * norm_w) * w3 * (w1 * w1 + w2 * w2) *
                                                                   (i_tan_norm * norm_w - 1.0) * 2.0 - (1.0 / (norm_w * norm_w * norm_w) * w3 * (w1 * w1 + w2 * w2) *
                                                                           (norm_w - s_norm_w) * (1.0 / 2.0)) / (c_norm_w - 1.0));


        }

        Eigen::Matrix<float, 6, 12> SE3_log;

        SE3_log.template block<3, 9>(0, 0) = J_LOG_T.block<3, 3>(0, 3) * JL;
        SE3_log.template block<3, 3>(0, 9) = J_LOG_T.block<3, 3>(0, 0);
        SE3_log.template block<3, 9>(3, 0) = JL;
        SE3_log.template block<3, 3>(3, 9) = Eigen::Matrix3f::Zero();

        return SE3_log;
    }

    static inline Eigen::Matrix<float, 12, 6> jacobianse3Exp(Eigen::Matrix4f& RT) {
        //J = [zeros(3, 3), -getskew(R(:, 1));
        //           zeros(3, 3), -getskew(R(:, 2));
        //           zeros(3, 3), -getskew(R(:, 3));
        //           eye(3), -getskew(T)];
        Eigen::Vector3f R1 = RT.template block<3, 1>(0, 0);
        Eigen::Vector3f R2 = RT.template block<3, 1>(0, 1);
        Eigen::Vector3f R3 = RT.template block<3, 1>(0, 2);
        Eigen::Vector3f T = RT.template block<3, 1>(0, 3);

        Eigen::Matrix<float, 12, 6> J;
        J.template block<3, 3>(0, 0).setZero();
        J.template block<3, 3>(0, 3) = getNegativeSkew(R1);

        J.template block<3, 3>(3, 0).setZero();
        J.template block<3, 3>(3, 3) = getNegativeSkew(R2);

        J.template block<3, 3>(6, 0).setZero();
        J.template block<3, 3>(6, 3) = getNegativeSkew(R3);

        J.template block<3, 3>(9, 0).setIdentity();
        J.template block<3, 3>(9, 3) = getNegativeSkew(T);

        return J;
    }

    /**
    // poseAtoB : basis change from A to B
    // covarianceInA : covariance in A basis
    // poseInA : pose in A basis associated with covarianceInA
    // return covariance in B frame
    */
    Eigen::Matrix<float, 6, 6> poseCovarianceAToB(Eigen::Matrix4f& poseAtoB, Eigen::Matrix4f& poseInA,
            Eigen::Matrix<float, 6, 6>& covarianceInA) {

        Eigen::Matrix4f poseInB = poseAtoB * poseInA * poseAtoB.inverse();
        Eigen::Matrix3f rotationInB = poseInB.template block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 9> JLOGSO3 = JacobianSO3Log(rotationInB);
        Eigen::Vector3f R1 = poseInA.template block<3, 1>(0, 0);
        Eigen::Vector3f R2 = poseInA.template block<3, 1>(0, 1);
        Eigen::Vector3f R3 = poseInA.template block<3, 1>(0, 2);

        Eigen::Matrix<float, 9, 3> JExpso3;
        JExpso3.template block<3, 3>(0, 0) = getNegativeSkew(R1);
        JExpso3.template block<3, 3>(3, 0) = getNegativeSkew(R2);
        JExpso3.template block<3, 3>(6, 0) = getNegativeSkew(R3);

        Eigen::Matrix<float, 9, 9> JCR;
        float Rc1_1 = poseAtoB(0, 0);
        float Rc1_2 = poseAtoB(0, 1);
        float Rc1_3 = poseAtoB(0, 2);

        float Rc2_1 = poseAtoB(1, 0);
        float Rc2_2 = poseAtoB(1, 1);
        float Rc2_3 = poseAtoB(1, 2);

        float Rc3_1 = poseAtoB(2, 0);
        float Rc3_2 = poseAtoB(2, 1);
        float Rc3_3 = poseAtoB(2, 2);

        float Tc1 = poseAtoB(0, 3);
        float Tc2 = poseAtoB(1, 3);
        float Tc3 = poseAtoB(2, 3);

        JCR(0, 0) = Rc1_1 * Rc1_1;
        JCR(0, 1) = Rc1_1 * Rc2_1;
        JCR(0, 2) = Rc1_1 * Rc3_1;
        JCR(0, 3) = Rc1_1 * Rc2_1;
        JCR(0, 4) = Rc2_1 * Rc2_1;
        JCR(0, 5) = Rc2_1 * Rc3_1;
        JCR(0, 6) = Rc1_1 * Rc3_1;
        JCR(0, 7) = Rc2_1 * Rc3_1;
        JCR(0, 8) = Rc3_1 * Rc3_1;

        JCR(1, 0) = Rc1_1 * Rc1_2;
        JCR(1, 1) = Rc1_1 * Rc2_2;
        JCR(1, 2) = Rc1_1 * Rc3_2;
        JCR(1, 3) = Rc1_2 * Rc2_1;
        JCR(1, 4) = Rc2_1 * Rc2_2;
        JCR(1, 5) = Rc2_1 * Rc3_2;
        JCR(1, 6) = Rc1_2 * Rc3_1;
        JCR(1, 7) = Rc2_2 * Rc3_1;
        JCR(1, 8) = Rc3_1 * Rc3_2;

        JCR(2, 0) = Rc1_1 * Rc1_3;
        JCR(2, 1) = Rc1_1 * Rc2_3;
        JCR(2, 2) = Rc1_1 * Rc3_3;
        JCR(2, 3) = Rc1_3 * Rc2_1;
        JCR(2, 4) = Rc2_1 * Rc2_3;
        JCR(2, 5) = Rc2_1 * Rc3_3;
        JCR(2, 6) = Rc1_3 * Rc3_1;
        JCR(2, 7) = Rc2_3 * Rc3_1;
        JCR(2, 8) = Rc3_1 * Rc3_3;

        JCR(3, 0) = Rc1_1 * Rc1_2;
        JCR(3, 1) = Rc1_2 * Rc2_1;
        JCR(3, 2) = Rc1_2 * Rc3_1;
        JCR(3, 3) = Rc1_1 * Rc2_2;
        JCR(3, 4) = Rc2_1 * Rc2_2;
        JCR(3, 5) = Rc2_2 * Rc3_1;
        JCR(3, 6) = Rc1_1 * Rc3_2;
        JCR(3, 7) = Rc2_1 * Rc3_2;
        JCR(3, 8) = Rc3_1 * Rc3_2;

        JCR(4, 0) = Rc1_2 * Rc1_2;
        JCR(4, 1) = Rc1_2 * Rc2_2;
        JCR(4, 2) = Rc1_2 * Rc3_2;
        JCR(4, 3) = Rc1_2 * Rc2_2;
        JCR(4, 4) = Rc2_2 * Rc2_2;
        JCR(4, 5) = Rc2_2 * Rc3_2;
        JCR(4, 6) = Rc1_2 * Rc3_2;
        JCR(4, 7) = Rc2_2 * Rc3_2;
        JCR(4, 8) = Rc3_2 * Rc3_2;

        JCR(5, 0) = Rc1_2 * Rc1_3;
        JCR(5, 1) = Rc1_2 * Rc2_3;
        JCR(5, 2) = Rc1_2 * Rc3_3;
        JCR(5, 3) = Rc1_3 * Rc2_2;
        JCR(5, 4) = Rc2_2 * Rc2_3;
        JCR(5, 5) = Rc2_2 * Rc3_3;
        JCR(5, 6) = Rc1_3 * Rc3_2;
        JCR(5, 7) = Rc2_3 * Rc3_2;
        JCR(5, 8) = Rc3_2 * Rc3_3;

        JCR(6, 0) = Rc1_1 * Rc1_3;
        JCR(6, 1) = Rc1_3 * Rc2_1;
        JCR(6, 2) = Rc1_3 * Rc3_1;
        JCR(6, 3) = Rc1_1 * Rc2_3;
        JCR(6, 4) = Rc2_1 * Rc2_3;
        JCR(6, 5) = Rc2_3 * Rc3_1;
        JCR(6, 6) = Rc1_1 * Rc3_3;
        JCR(6, 7) = Rc2_1 * Rc3_3;
        JCR(6, 8) = Rc3_1 * Rc3_3;

        JCR(7, 0) = Rc1_2 * Rc1_3;
        JCR(7, 1) = Rc1_3 * Rc2_2;
        JCR(7, 2) = Rc1_3 * Rc3_2;
        JCR(7, 3) = Rc1_2 * Rc2_3;
        JCR(7, 4) = Rc2_2 * Rc2_3;
        JCR(7, 5) = Rc2_3 * Rc3_2;
        JCR(7, 6) = Rc1_2 * Rc3_3;
        JCR(7, 7) = Rc2_2 * Rc3_3;
        JCR(7, 8) = Rc3_2 * Rc3_3;

        JCR(8, 0) = Rc1_3 * Rc1_3;
        JCR(8, 1) = Rc1_3 * Rc2_3;
        JCR(8, 2) = Rc1_3 * Rc3_3;
        JCR(8, 3) = Rc1_3 * Rc2_3;
        JCR(8, 4) = Rc2_3 * Rc2_3;
        JCR(8, 5) = Rc2_3 * Rc3_3;
        JCR(8, 6) = Rc1_3 * Rc3_3;
        JCR(8, 7) = Rc2_3 * Rc3_3;
        JCR(8, 8) = Rc3_3 * Rc3_3;

        Eigen::Matrix<float, 3, 9> JCRT;
        JCRT(0, 0) = Rc1_1 * Tc1;
        JCRT(0, 1) = Rc2_1 * Tc1;
        JCRT(0, 2) = Rc3_1 * Tc1;
        JCRT(0, 3) = Rc1_1 * Tc2;
        JCRT(0, 4) = Rc2_1 * Tc2;
        JCRT(0, 5) = Rc3_1 * Tc2;
        JCRT(0, 6) = Rc1_1 * Tc3;
        JCRT(0, 7) = Rc2_1 * Tc3;
        JCRT(0, 8) = Rc3_1 * Tc3;
        JCRT(1, 0) = Rc1_2 * Tc1;
        JCRT(1, 1) = Rc2_2 * Tc1;
        JCRT(1, 2) = Rc3_2 * Tc1;
        JCRT(1, 3) = Rc1_2 * Tc2;
        JCRT(1, 4) = Rc2_2 * Tc2;
        JCRT(1, 5) = Rc3_2 * Tc2;
        JCRT(1, 6) = Rc1_2 * Tc3;
        JCRT(1, 7) = Rc2_2 * Tc3;
        JCRT(1, 8) = Rc3_2 * Tc3;
        JCRT(2, 0) = Rc1_3 * Tc1;
        JCRT(2, 1) = Rc2_3 * Tc1;
        JCRT(2, 2) = Rc3_3 * Tc1;
        JCRT(2, 3) = Rc1_3 * Tc2;
        JCRT(2, 4) = Rc2_3 * Tc2;
        JCRT(2, 5) = Rc3_3 * Tc2;
        JCRT(2, 6) = Rc1_3 * Tc3;
        JCRT(2, 7) = Rc2_3 * Tc3;
        JCRT(2, 8) = Rc3_3 * Tc3;

        Eigen::Matrix3f JCT;
        JCT(0, 0) = Rc1_1;
        JCT(0, 1) = Rc2_1;
        JCT(0, 2) = Rc3_1;
        JCT(1, 0) = Rc1_2;
        JCT(1, 1) = Rc2_2;
        JCT(1, 2) = Rc3_2;
        JCT(2, 0) = Rc1_3;
        JCT(2, 1) = Rc2_3;
        JCT(2, 2) = Rc3_3;

        Eigen::Matrix<float, 6, 6> J_transform;
        J_transform.template block<3, 3>(0, 0).setZero();
        J_transform.template block<3, 3>(0, 3) = JLOGSO3 * JCR * JExpso3;
        J_transform.template block<3, 3>(3, 0) = JCT;
        J_transform.template block<3, 3>(3, 3) = JCRT * JExpso3;

        return J_transform * covarianceInA * J_transform.transpose();
    }

    Eigen::Matrix<float, 6, 6> twistCovarianceAtoB(Eigen::Matrix4f& poseAtoB, Eigen::Matrix<float, 6, 1>& twistInA,
            Eigen::Matrix<float, 6, 6>& twistCovarianceInA) {

        Eigen::Matrix4f pose_in_A = Sophus::SE3f::exp(twistInA).matrix();

        Eigen::Matrix<float, 12, 6> JExpse3 = jacobianse3Exp(pose_in_A);

        float Rc1_1 = poseAtoB(0, 0);
        float Rc1_2 = poseAtoB(0, 1);
        float Rc1_3 = poseAtoB(0, 2);

        float Rc2_1 = poseAtoB(1, 0);
        float Rc2_2 = poseAtoB(1, 1);
        float Rc2_3 = poseAtoB(1, 2);

        float Rc3_1 = poseAtoB(2, 0);
        float Rc3_2 = poseAtoB(2, 1);
        float Rc3_3 = poseAtoB(2, 2);

        float Tc1 = poseAtoB(0, 3);
        float Tc2 = poseAtoB(1, 3);
        float Tc3 = poseAtoB(2, 3);

        Eigen::Matrix<float, 12, 12> JC;

        JC(0, 0) = Rc1_1 * Rc1_1;
        JC(0, 1) = Rc1_1 * Rc2_1;
        JC(0, 2) = Rc1_1 * Rc3_1;
        JC(0, 3) = Rc1_1 * Rc2_1;
        JC(0, 4) = Rc2_1 * Rc2_1;
        JC(0, 5) = Rc2_1 * Rc3_1;
        JC(0, 6) = Rc1_1 * Rc3_1;
        JC(0, 7) = Rc2_1 * Rc3_1;
        JC(0, 8) = Rc3_1 * Rc3_1;
        JC(1, 0) = Rc1_1 * Rc1_2;
        JC(1, 1) = Rc1_1 * Rc2_2;
        JC(1, 2) = Rc1_1 * Rc3_2;
        JC(1, 3) = Rc1_2 * Rc2_1;
        JC(1, 4) = Rc2_1 * Rc2_2;
        JC(1, 5) = Rc2_1 * Rc3_2;
        JC(1, 6) = Rc1_2 * Rc3_1;
        JC(1, 7) = Rc2_2 * Rc3_1;
        JC(1, 8) = Rc3_1 * Rc3_2;
        JC(2, 0) = Rc1_1 * Rc1_3;
        JC(2, 1) = Rc1_1 * Rc2_3;
        JC(2, 2) = Rc1_1 * Rc3_3;
        JC(2, 3) = Rc1_3 * Rc2_1;
        JC(2, 4) = Rc2_1 * Rc2_3;
        JC(2, 5) = Rc2_1 * Rc3_3;
        JC(2, 6) = Rc1_3 * Rc3_1;
        JC(2, 7) = Rc2_3 * Rc3_1;
        JC(2, 8) = Rc3_1 * Rc3_3;
        JC(3, 0) = Rc1_1 * Rc1_2;
        JC(3, 1) = Rc1_2 * Rc2_1;
        JC(3, 2) = Rc1_2 * Rc3_1;
        JC(3, 3) = Rc1_1 * Rc2_2;
        JC(3, 4) = Rc2_1 * Rc2_2;
        JC(3, 5) = Rc2_2 * Rc3_1;
        JC(3, 6) = Rc1_1 * Rc3_2;
        JC(3, 7) = Rc2_1 * Rc3_2;
        JC(3, 8) = Rc3_1 * Rc3_2;
        JC(4, 0) = Rc1_2 * Rc1_2;
        JC(4, 1) = Rc1_2 * Rc2_2;
        JC(4, 2) = Rc1_2 * Rc3_2;
        JC(4, 3) = Rc1_2 * Rc2_2;
        JC(4, 4) = Rc2_2 * Rc2_2;
        JC(4, 5) = Rc2_2 * Rc3_2;
        JC(4, 6) = Rc1_2 * Rc3_2;
        JC(4, 7) = Rc2_2 * Rc3_2;
        JC(4, 8) = Rc3_2 * Rc3_2;
        JC(5, 0) = Rc1_2 * Rc1_3;
        JC(5, 1) = Rc1_2 * Rc2_3;
        JC(5, 2) = Rc1_2 * Rc3_3;
        JC(5, 3) = Rc1_3 * Rc2_2;
        JC(5, 4) = Rc2_2 * Rc2_3;
        JC(5, 5) = Rc2_2 * Rc3_3;
        JC(5, 6) = Rc1_3 * Rc3_2;
        JC(5, 7) = Rc2_3 * Rc3_2;
        JC(5, 8) = Rc3_2 * Rc3_3;
        JC(6, 0) = Rc1_1 * Rc1_3;
        JC(6, 1) = Rc1_3 * Rc2_1;
        JC(6, 2) = Rc1_3 * Rc3_1;
        JC(6, 3) = Rc1_1 * Rc2_3;
        JC(6, 4) = Rc2_1 * Rc2_3;
        JC(6, 5) = Rc2_3 * Rc3_1;
        JC(6, 6) = Rc1_1 * Rc3_3;
        JC(6, 7) = Rc2_1 * Rc3_3;
        JC(6, 8) = Rc3_1 * Rc3_3;
        JC(7, 0) = Rc1_2 * Rc1_3;
        JC(7, 1) = Rc1_3 * Rc2_2;
        JC(7, 2) = Rc1_3 * Rc3_2;
        JC(7, 3) = Rc1_2 * Rc2_3;
        JC(7, 4) = Rc2_2 * Rc2_3;
        JC(7, 5) = Rc2_3 * Rc3_2;
        JC(7, 6) = Rc1_2 * Rc3_3;
        JC(7, 7) = Rc2_2 * Rc3_3;
        JC(7, 8) = Rc3_2 * Rc3_3;
        JC(8, 0) = Rc1_3 * Rc1_3;
        JC(8, 1) = Rc1_3 * Rc2_3;
        JC(8, 2) = Rc1_3 * Rc3_3;
        JC(8, 3) = Rc1_3 * Rc2_3;
        JC(8, 4) = Rc2_3 * Rc2_3;
        JC(8, 5) = Rc2_3 * Rc3_3;
        JC(8, 6) = Rc1_3 * Rc3_3;
        JC(8, 7) = Rc2_3 * Rc3_3;
        JC(8, 8) = Rc3_3 * Rc3_3;
        JC(9, 0) = Rc1_1 * Tc1;
        JC(9, 1) = Rc2_1 * Tc1;
        JC(9, 2) = Rc3_1 * Tc1;
        JC(9, 3) = Rc1_1 * Tc2;
        JC(9, 4) = Rc2_1 * Tc2;
        JC(9, 5) = Rc3_1 * Tc2;
        JC(9, 6) = Rc1_1 * Tc3;
        JC(9, 7) = Rc2_1 * Tc3;
        JC(9, 8) = Rc3_1 * Tc3;
        JC(9, 9) = Rc1_1;
        JC(9, 10) = Rc2_1;
        JC(9, 11) = Rc3_1;
        JC(10, 0) = Rc1_2 * Tc1;
        JC(10, 1) = Rc2_2 * Tc1;
        JC(10, 2) = Rc3_2 * Tc1;
        JC(10, 3) = Rc1_2 * Tc2;
        JC(10, 4) = Rc2_2 * Tc2;
        JC(10, 5) = Rc3_2 * Tc2;
        JC(10, 6) = Rc1_2 * Tc3;
        JC(10, 7) = Rc2_2 * Tc3;
        JC(10, 8) = Rc3_2 * Tc3;
        JC(10, 9) = Rc1_2;
        JC(10, 10) = Rc2_2;
        JC(10, 11) = Rc3_2;
        JC(11, 0) = Rc1_3 * Tc1;
        JC(11, 1) = Rc2_3 * Tc1;
        JC(11, 2) = Rc3_3 * Tc1;
        JC(11, 3) = Rc1_3 * Tc2;
        JC(11, 4) = Rc2_3 * Tc2;
        JC(11, 5) = Rc3_3 * Tc2;
        JC(11, 6) = Rc1_3 * Tc3;
        JC(11, 7) = Rc2_3 * Tc3;
        JC(11, 8) = Rc3_3 * Tc3;
        JC(11, 9) = Rc1_3;
        JC(11, 10) = Rc2_3;
        JC(11, 11) = Rc3_3;

        JC.block<9, 3>(0, 9).setZero();

        Eigen::Matrix4f pose_in_B = poseAtoB * pose_in_A * poseAtoB.inverse();

        Eigen::Matrix<float, 6, 12> JLOGSE3 = jacobianSE3Log(pose_in_B);

        Eigen::Matrix<float, 6, 6> J_Transform = JLOGSE3 * JC * JExpse3;

        return J_Transform * twistCovarianceInA * J_Transform.transpose();

    }

}
