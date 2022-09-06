# tracking thread
## 经验参数
1. 期望提取的特征点数
2. 金字塔层数
3. 关键帧设置条件
4. 跟踪成功的判断条件
## 运行结果不同的原因
1. Ransac的随机性（初始化时随机数，计算EPNEP，使用控制点表示3D坐标）
2. 多线程之间的随机性，中断某一线程导致位姿、地图点更新操作延迟

## 间接法 、光流法、直接法对于雅各比矩阵的理解

<font color=yellow> 光流法、直接法都基于一个空间点的灰度不变假设（当且相机帧数较大）</font>

* 光流法：假设所有像素都有相同运动，优化像素的运动速度，

![image-20220906165710074](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906165710074.png)

已知右边灰度关于时间的导数表示为两张图片之间的灰度差，这样可以构建最小二乘问题，求解像素在x, y 方向上的运动速度。

这样跟踪到像素点之后，就利用对极几何、或者PNP之类的进行跟踪（理论上可以光流所有点，这样可以实现稠密重建）

* 直接法:直接依据灰度不变假设，构建像素点之间的光度误差，直接优化相应的位姿（跳过匹配位姿过程）

  * 三维点对增量位姿的导数，扰动模型

  <cecnter>

  * ![image-20220906180837217](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906180837217.png)

  </center>		

扰动模型记忆：增量在哪边，对应的左右扰动，同时基于增量计算雅各比，再将原始量带入计算最后乘增量。

### 重投影误差的雅各比

![image-20220906202622622](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906202622622.png)

![image-20220906202650912](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906202650912.png)

![image-20220906203636067](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906203636067.png)

其中，$e = || u_i - u||^2 \quad s * u = K * P^\prime$

相机坐标系下的位置关于扰动位姿的求导

[**I**,**-(P')^**]

[ 1, 0, 0, 0, Z, -Y ]

[ 0, 1, 0, -Z, 0, X ]

[ 0, 0, 1, Y, -X, 0 ]

[ 0, 0, 0, 0, 0, 1 ]

两式相乘得到误差关于位姿的导数：

![image-20220906203741098](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906203741098.png)



### 光度误差的雅各比

![image-20220906204155719](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906204155719.png)

对位姿增量进行求导，将位姿增量泰勒展开

![image-20220906204544137](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906204544137.png)

由于误差中和位姿相关的只包括在后面的一部分，将$I_2$进行泰勒展开，

![image-20220906204939617](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906204939617.png)

此处的U表示增量部分在相机2下的像素坐标，Q表示相机2下的坐标，后面两者的乘积与上面类似最后得到：

![image-20220906205252616](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906205252616.png)

最后在获得相机2下在U处的像素梯度

![image-20220906205511444](C:\Users\way\AppData\Roaming\Typora\typora-user-images\image-20220906205511444.png)

最后将这个作为非线性优化部分增量方程的更新，求出增量$\delta x$，更新位姿，开启下轮迭代
