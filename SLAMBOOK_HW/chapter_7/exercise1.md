### ORB\SIFT\SURF 特征点的优劣：
***
1. ORB:Oriented FAST and Rotated BRIEF,由两部分组成：快速特征点提取以及描述算法；提取算法：主要描述图片像素灰度值的变化，
首先计算局部点的亮度，以像素点为中心，选取一定半径上的圆周，计算圆周上个点的灰度值，当出现一定数量的连续较高、较低亮度时，则将
该点视为特征点；描述算法中，随机提取特点附近128对像素，比较之，获得128维的特征向量。

    特点：计算速度块，实时性较好，尺度变化性较弱，适用于场景变化不明显，但需要高速计算（VSLAM）。
    ***
2.  SIFT:Scale-Invariant Feature Transform, 由两部分组成：快速特征点提取以及描述算法;特征点检出：将图像放大、缩小几倍后进行
高斯模糊操作，找到变化较大的点即为特征点；特征点描述：选定16X6区域，划分为4X4的子区域，根据梯度方向（上下左右，左上、右上、左下、右下）
划分成8个区间，计算得到子区域内的4X4X8的128维向量，向量元素表示每个梯度方向上的区间权值，并对特征向量进行归一化，，得到邻域关键点的主方向
，据此将领域旋转至特定方向，，并根据领域各像素大小缩放至指定尺度。

    特点：特征稳定，视角变换、噪声具有一定程度的稳定性；实时性较低，边缘光滑目标的特征点提取能力较弱。
    ***
3. SURF:Speeded Up Robust Features,改进了特征的提取和描述方式，用一种更为高效的方式完成特征的提取和描述。通过引入积分图和箱式
滤波器加速Hessian矩阵的计算，通过比较Hessian矩阵行列式的大小来选择特征点的位置和尺度。关于特征点的描述，SURF基于Harr小波设定了特征
点的主方向，并充分利用积分图构建64维的特征描述子。

    特点：基于SIFT的算法改进，效率提升一个量级。求取特征点主方向时过分局部区域像素点的梯度方向，误差较大，描述子不准确
    ***
* 上述三种方法在提取特征点时，都应实现方向旋转和尺度变换的的不变性。
* [参考文章](https://zhuanlan.zhihu.com/p/389420864)