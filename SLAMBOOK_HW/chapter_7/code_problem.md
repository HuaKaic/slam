## some bugs in chapter_7   (PS:fixed up already)
### 程序执行时出现：malloc(): memory corruption(fast)
1. 数组、容器出现越界，索引超出范围；
2. g2o求解器选择不合理，在练习7中将CSparse换成Dense模块解决上述问题；
***
***
### Sophus::SE3 has no member named 'rotationmatrix'
* change to rotation_matrix
***
***
### 在BA过程中，出现求解旋转矩阵迭代过程中，临时中断，导致所求矩阵误差较大
* 换用别的迭代方法，练习中将高斯-牛顿法换成列文-伯格解决问题；
***
***
### 对于相机位姿节点，可以选择自定义，也可以选择```g2o::VertexExpmap()``` 后者估计量是一个六自由度的向量，要按照相应的函数转化为旋转矩阵和平移向量；```Eigen::ISometry3d()''' 欧式变换矩阵，4x4;
