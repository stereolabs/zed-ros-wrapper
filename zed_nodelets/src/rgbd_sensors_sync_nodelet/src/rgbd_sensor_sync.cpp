///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
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

#include "rgbd_sensor_sync.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

namespace zed_nodelets {

RgbdSensorsSyncNodelet::RgbdSensorsSyncNodelet() {

}

RgbdSensorsSyncNodelet::~RgbdSensorsSyncNodelet() {
    if(mApproxFullSync)
        delete mApproxFullSync;
    if(mApproxRgbdImuSync)
        delete mApproxRgbdImuSync;
    if(mApproxRgbdSync)
        delete mApproxRgbdSync;

    if(mExactFullSync)
        delete mExactFullSync;
    if(mExactRgbdImuSync)
        delete mExactRgbdImuSync;
    if(mExactRgbdSync)
        delete mExactRgbdSync;
}

void RgbdSensorsSyncNodelet::onInit() {
    // Node handlers
    mNh = getMTNodeHandle();
    mNhNs = getMTPrivateNodeHandle();

#ifndef NDEBUG
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

    readParameters();

    if( mUseApproxSync ) {
        if( mUseImu && mUseMag ) {

        } else if( mUseImu ) {

        } else if( mUseMag ) {

        } else {

        }

    } else {

    }
}

void RgbdSensorsSyncNodelet::readParameters() {
    NODELET_INFO("*** PARAMETERS [%s]***",  getName().c_str());

    mNhNs.param("approx_sync", mUseApproxSync, mUseApproxSync);
    mNhNs.param("queue_size", mQueueSize, mQueueSize);
    mNhNs.param("sub_imu", mUseImu, mUseImu);
    mNhNs.param("sub_mag", mUseMag, mUseMag);

    NODELET_INFO(" * approx_sync -> %s",, mUseApproxSync?"true":"false");
    NODELET_INFO(" * queue_size  -> %d", getName().c_str(), mQueueSize);
    NODELET_INFO(" * sub_imu -> %s", getName().c_str(), mUseImu?"true":"false");
    NODELET_INFO(" * sub_mag -> %s", getName().c_str(), mUseMag?"true":"false");
}

}
