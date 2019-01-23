/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef _IBA_DATATYPE_H_
#define _IBA_DATATYPE_H_

#include <IBA_config.h>

#define IBA_SERIAL_NONE   0x00000000
#define IBA_SERIAL_LBA    0x00000001
#define IBA_SERIAL_GBA    0x00000100

#define IBA_VERBOSE_NONE  0x00000000
#define IBA_VERBOSE_LBA   0x00000001
#define IBA_VERBOSE_GBA   0x00000100

#define IBA_DEBUG_NONE    0x00000000
#define IBA_DEBUG_LBA     0x00000001
#define IBA_DEBUG_GBA     0x00000100

#define IBA_HISTORY_NONE  0x00000000
#define IBA_HISTORY_LBA   0x00000001
#define IBA_HISTORY_GBA   0x00000100

namespace IBA {

struct Intrinsic {
  float fx, fy;   // focal length焦距
  float cx, cy;   // optic center光学中心
  float ds[8];    // distortion parameters畸变系数
};

struct Calibration {
  int w, h;       // image resolution图像分辨率
  bool fishEye;   // fish eye distortion model鱼眼扭曲模型
  float Tu[3][4]; // X_cam = Tu * X_imu
  float ba[3];    // initial acceleration bias初始加速度偏差
  float bw[3];    // initial gyroscope bias初始陀螺仪偏差
  //float sa[3];
  Intrinsic K;    // intrinsic parameters内参
#ifdef CFG_STEREO
  float Tr[3][4]; // X_left = Tr * X_right
  Intrinsic Kr;   // intrinsic parameters for right camera右相机的内在参数
#endif
};

struct CameraPose {
  float R[3][3];  // rotation matrix, R[0][0] = FLT_MAX for unknown camera pose旋转矩阵，R [0] [0] = FLT_MAX，用于未知的相机姿势
  float p[3];     // position位置
};                // for a 3D point in world frame X, its coordinate in camera frame is obtained by R * (X - p)
                  //对于世界框架X中的3D点，其相机框架中的坐标由R *（X  -  p）获得

struct CameraPoseCovariance {
  float S[6][6];  // position + rotation位置+旋转
                  // p = \hat p + \tilde p
                  // R = \hat R * exp(\tilde\theta)
};

struct CameraIMUState {
  CameraPose C;   // camera pose相机姿势
  float v[3];     // velocity, v[0] = FLT_MAX for unknown velocity
  float ba[3];    // acceleration bias, ba[0] = FLT_MAX for unknown acceleration bias
  float bw[3];    // gyroscope bias, bw[0] = FLT_MAX for unknown gyroscope bias
};

struct Depth {
  float d;   // inverse depth, d = 0 for unknown depth反深度，d = 0表示未知深度
  float s2;  // variance 方差
};

struct Point2D {
  float x[2];     // feature location in the original image原始图像中的特征位置
  float S[2][2];  // covariance matrix in the original image原始图像中的协方差矩阵
};

struct Point3D {
  int idx;    // global point index全局点下标
  float X[3]; // 3D position, X[0] = FLT_MAX for unknown 3D position
};

struct MapPointMeasurement {
  union {
    int iFrm; // frame ID帧ID
    int idx;  // global point ID全局点ID
  };
  inline bool operator < (const MapPointMeasurement &X) const {
    return iFrm < X.iFrm
#ifdef CFG_STEREO
//#if 1
        || iFrm <= X.iFrm && !right && X.right
#endif
        ;
  }
  Point2D x;
#ifdef CFG_STEREO
//#if 1
  ubyte right;
#endif
};

struct MapPoint {
  Point3D X;
  std::vector<MapPointMeasurement> zs;
};

struct FeatureTrack {
  int idx;
  Point2D x;
};

struct IMUMeasurement {
  float a[3];     // acceleration加速度
  float w[3];     // gyroscope陀螺仪
  float t;        // timestamp时间戳
};

struct CurrentFrame {
  int iFrm;                             // frame index帧索引
  CameraIMUState C;                     // initial camera/IMU state of current frame当前帧的初始相机/ IMU状态
  std::vector<MapPointMeasurement> zs;  // feature measurements of current frame当前帧的特征测量
  std::vector<IMUMeasurement> us;       // IMU measurements between last frame and current frame;最后一帧和当前帧之间的IMU测量;
                                        // the timestamp of first IMU must be the same as last frame第一个IMU的时间戳必须与最后一帧相同
  float t;                              // timestamp of current frame, should be greater than the timestamp of last IMU当前帧的时间戳，应大于上一个IMU的时间戳
  Depth d;                              // a rough depth estimate for current frame当前帧的粗略深度估计
                                        // (e.g. average over all visible points in current frame)（例如，当前帧中所有可见点的平均值）
  std::string fileName;                 // image file name, just for visualization图像文件名，仅用于可视化
#ifdef CFG_STEREO
//#if 1
  std::string fileNameRight;
#endif
};

struct KeyFrame {
  int iFrm;                             // frame index, -1 for invalid keyframe帧索引，-1表示无效的关键帧
  CameraPose C;                         // initial camera pose of keyframe关键帧的初始相机姿势
  std::vector<MapPointMeasurement> zs;  // feature measurements of keyframe关键帧的特征测量
  std::vector<MapPoint> Xs;             // new map points新的地图点
  Depth d;                              // a rough depth estimate粗略的深度估计
};

struct SlidingWindow {
  std::vector<int> iFrms;           // frame indexes of those sliding window frames whose滑动窗口框架的索引
                                    // camera/IMU state has been updated since last call自上次通话以来，相机/ IMU状态已更新
  std::vector<CameraIMUState> CsLF; // camera/IMU states corresponding to iFrms对应于iFrms的摄像机/ IMU状态
  std::vector<int> iFrmsKF;         // frame indexes of those keyframes whose camera pose相机姿势的那些关键帧的帧索引
                                    // has been updated since last call自上次通话以来已更新
  std::vector<CameraPose> CsKF;     // camera poses corresponding to iFrmsKF相机姿势对应iFrmsKF
  std::vector<Point3D> Xs;          // updated 3D points since last call自上次通话后更新的3D点数
#ifdef CFG_CHECK_REPROJECTION
  std::vector<std::pair<float, float> > esLF, esKF;
#endif
};

struct RelativeConstraint {
  int iFrm1, iFrm2;
  CameraPose T;       // X2 = T * X1 = R * (X1 - p)
  CameraPoseCovariance S;      
};

struct Error {
  float ex;                         // feature reprojection error特征点重投影误差
  float eur, eup, euv, euba, eubw;  // IMU delta error   IMU delta误差
  float edr, edp;                   // drift error compared to ground truth与地面实况相比的漂移误差
};

struct Time {
  float t;
  int n;
};

}  // namespace IBA

#endif  //  _IBA_DATATYPE_H_
