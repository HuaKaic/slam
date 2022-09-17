# problems in my project

## kinect camera

在深度图和RGB图都是1024 * 768 分辨率输入下，发现经常跟丢。分析之后发现在深度图左右两侧存在无效的深度信息。

在另一个分辨率640 * 435输入下，不能正常工作。只有彩色图中间部分存在地图点

原因分析：

* 由于深度相机和彩色相机视场角存在差异，所以实际效果取决于两者视场角的重合度。在利用内置函数配对深度图和彩色图时，两者分辨率出现差异（彩色图为720 * 1280，深度图为640 * 576），对齐后分辨率为1024 * 768.为此，可以改用宽视角模式，提高深度图的视角范围（120 * 120）以覆盖所有的彩色图。（但同时深度范围减小）
* 采用上述改进方案后，依旧无法改善相机跟丢的问题（可能说分辨率过高，导致提取到一些细节特征，特征不稳定，容易受到光照影响）解决的办法，由于分别率差异，增加orb特征数量来弥补这些分辨率差异，但最终的跟踪效果不尽如人意。
* 分析orbslam2源码的输入分辨率为640 * 480 并且计算八层金字塔，提取特征数为1000.在高分辨率输入下，增加特征数以及金字塔层数来获得与原版类似的分辨率。同时增加局部BA阶段的关键帧和地图点数量，通过在共视图中寻找一定距离内的关键帧。

# tips in my project

topic 修改

<font color = green>**/rgb_to_depth/image_raw  /depth/image_raw**</font>

修改相机的畸变参数个数

orb slam原本只能读取5个相机畸变参数：<font color = blue> k1，k2，p1，p2，k3 </font> 但是 azure-kinect有八个相机畸变参数：<font color = blue> k1，k2，k3，k4，k5，k6，p1，p2 </font>

在 tracking.cc中修改：

```
const float k3 = fSettings["Camera.k3"];
if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
```

```
const float k4 = fSettings["Camera.k4"];
    if(k4!=0)
    {
        DistCoef.resize(6);
        DistCoef.at<float>(5) = k4;
    }
    const float k5 = fSettings["Camera.k5"];
    if(k5!=0)
    {
        DistCoef.resize(7);
        DistCoef.at<float>(6) = k5;
    }
    const float k6 = fSettings["Camera.k6"];
    if(k6!=0)
    {
        DistCoef.resize(8);
        DistCoef.at<float>(7) = k6;
    }
```

```
if(DistCoef.rows==8)
    {
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- k4: " << DistCoef.at<float>(5) << endl;
        cout << "- k5: " << DistCoef.at<float>(6) << endl;
        cout << "- k6: " << DistCoef.at<float>(7) << endl;
    }
```

修改相机yaml文件

```
Camera.fx: 505.076
Camera.fy: 505.239
Camera.cx: 319.893
Camera.cy: 328.057

Camera.k1: 1.12383
Camera.k2: 0.629452
Camera.p1: 6.53274e-05
Camera.p2: -0.000118945
Camera.k3: 0.035496
Camera.k4: 1.46064
Camera.k5: 0.950368
Camera.k6: 0.180062

Camera.width: 640
Camera.height: 576

Camera.fps: 30.0
DepthMapFactor: 1.0
```
## Eigen 编译错误
![图片](https://user-images.githubusercontent.com/92136649/190848884-a96c079c-4b36-42df-8936-cfff90ee595c.png)
解决方法：cmakelists find_packages(Eigen3 3.1.0 REQUIRED) ---> find_packags(Eigen3 3.1.0 REQUIRED NO_MODULES)

