#ifndef SL_COV_TRANSF_H
#define SL_COV_TRANSF_H

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

#include <Eigen/Eigen>

namespace sl_tools {

    /** \brief Convert pose covariance from frame A to frame B
     *  \param poseAtoB : basis change from A to B
     *  \param poseInA : pose in A basis associated with covarianceInA
     *  \param covarianceInA : covariance in A basis
     */
    Eigen::Matrix<float, 6, 6> poseCovarianceAToB(Eigen::Matrix4f& poseAtoB, Eigen::Matrix4f& poseInA,
            Eigen::Matrix<float, 6, 6>& covarianceInA);

    /** \brief Convert twist covariance from frame A to frame B
     *  \param poseAtoB : basis change from A to B
     *  \param twistInA : twist in A basis associated with covarianceInA
     *  \param covarianceInA : covariance in A basis
     */
    Eigen::Matrix<float, 6, 6> twistCovarianceAtoB(Eigen::Matrix4f& poseAtoB, Eigen::Matrix<float, 6, 1>& twistInA,
            Eigen::Matrix<float, 6, 6>& twistCovarianceInA);

}
#endif
