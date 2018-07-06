# calibration with camera and velodyne

相机和激光雷达之间的标定，是为了确定相机坐标系到激光雷达坐标系的转换矩阵，包含六个维度，即三个旋转角度和三个平移量。  

## 原理

In this section, we only discuss the camera’s extrinsic calibration. There are two ways to calculate the extrinsic
parameters, euler angles and translations along XYZ axes:
• Simultaneously optimize these six parameters.
• Calculate rotation matrix first, and then optimize translations.
After intrinsic calibration, we could get the position and orientation of each grabbed chessboard in camera
coordinate. Meanwhile, we have extracted the points on each grabbed chessboard from Velodyne point-cloud.
Therefore, our optimization objective is to align these planes by tuning the camera’s extrinsic parameters.
(1) For the first way, the objective function is shown as Eq.1. This optimization is relatively slow and may
converge to local minimum without global optimization.  

<div align="center"><img src="picture/Screenshot from 2018-07-06 14-42-00.png" /></div>

Where,
• (α, β, γ) : euler angles
• (x, y, z) : translations
• R(α, β, γ) : rotation matrix
• T(x, y, z) : translation vector
• p i : ith chessboard’s position in camera coordinate
• n 0 i : ith chessboard’s normal vector in camera coordinate
• q i,j : ith chessboard’s jth point in Velodyne coordinate  


(2) For the second way, the calculation of rotation matrix is shown as Eq.2.  
<div align="center"><img src="picture/Screenshot from 2018-07-06 14-46-22.png" /></div>

Where,
• R : rotation matrix
• N : matrix formed by stacking all normals of grabbed chessboards in camera coordinate.
• M : matrix formed by stacking all normals of grabbed chessbaords in Velodyne coordinate.
Then the objective function for translations optimization is shown as Eq.3. This optimization is faster and
converges to global minimum more easily than Eq.1.  
<div align="center"><img src="picture/Screenshot from 2018-07-06 14-46-28.png" /></div>  

总的来说，第一种方法是使用迭代优化，对6个未知数进行优化，代价函数为标定板点云的重投影误差;第二种方法是先用标定板的平面的法向量求出旋转矩阵的解析解，再用重投影误差最小化的优化方法优化平移矩阵。
autoware里面更推荐第二种方法。

## autoware标定软件CalibrationToolkit

用户手册参考CalibrationToolkit_Manual.pdf。它提供相机标定、相机到激光雷达的标定。  
在runtime_manager里的使用方法参考Autoware_TierIV_Academy_v1.1.pdf的calibration章节。

标定步骤：  
1、对输入的激光雷达数据和摄像头数据进行记录，保存为rosbag包，需要包含point_raw 和image_raw包  
2、play 保存的rosbag，打开CalibrationToolkit软件，进行标定。

## autoware_camera_lidar_calibrator

该包的相机与激光雷达的标定方法如下：  
使用人工点击棋盘格的点进行相机与激光雷达的点的匹配，然后调用opencv中的solvepnp库进行标定，求解变换矩阵，最少需要8个点。

## calibration_camera_lidar

该包的相机与激光雷达的标定方法如下：  
使用CalibrationToolkit_Manual.pdf文档里提到的最小化重投影误差的方法，利用nlopt库进行非线性优化。
