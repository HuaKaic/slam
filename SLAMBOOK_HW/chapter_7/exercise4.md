*  FLANN：fast libraries for approximate nearest neighbors;近似最近邻搜索库了，利用KDtree，
以特征点描述子的K维矩阵构建二叉特征树，利用了回溯算法和分割最大方差维度，在搜索距离最近的描述子过程中，充分利用了之前的信息
，降低了时间复杂度。
***
* 加速匹配的方法还有GPU以及可编程阵列逻辑。
