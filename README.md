# 实时激光雷达点云重建

## 项目说明

### 环境配置

1. 在 Ubuntu 64-bit 18.04 系统下使用该项目.

2. 安装 ROS Melodic 环境, 参见 <a href="http://wiki.ros.org/melodic/Installation/Ubuntu">ROS 官方文档</a>.

3. 安装 Ceres Solver, 参见 <a href="http://ceres-solver.org/installation.html">Ceres Solver 官方文档</a>.

4. 安装 PCL, 参见 <a href="https://pointclouds.org/downloads/#linux">PCL 官方文档</a>.

5. 安装 Anaconda, 参见 <a href="https://www.anaconda.com/products/distribution#macos">Anaconda 官方文档</a>.

6. 修改 ROS 脚本文件的超时时间, 使得在程序运行结束时可以保存点云结果.

   ```shell
   # 打开文件 nodeprocess.py
   sudo gedit /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/nodeprocess.py
   
   # 修改 nodeprocess.py 文件中的如下语句:
   _TIMEOUT_SIGINT = 15.0 
   # 将时间从 15.0s 改为 60.0 秒或更多
   _TIMEOUT_SIGINT = 60.0
   ```

### 构建项目

```shell
# 先进入项目目录
cd LidarPointCloudReconstruction

# 配置 anaconda 环境, 如果不需要泊松重建部分可以忽略此项
conda env create -f environment.yml
conda activate denseRos

# 创建工作目录
mkdir -p ~/catkin_ws/src

# 将项目文件夹拷贝到 ~/catkin_ws/src 文件夹中
cd .. 
cp -r LidarPointCloudReconstruction ~/catkin_ws/src
ln -s ~/catkin_ws/src/LidarPointCloudReconstruction

# 构建项目
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release # 如果不需要泊松重建,否则运行下一句
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/home/vcc/anaconda3/envs/puma/bin/python

```

### 下载数据包

下载 [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) 数据包，并保存到`data`文件夹中。

### 项目运行

#### 项目示例运行

```shell
# 项目注册到 ROS 中
source ~/catkin_ws/devel/setup.bash

# 运行 Launch 文件
roslaunch simple_frame mapping.launch 
```

点击运行之后,如果出现如下界面,说明配置成功.![img](img/sendpix0.jpg)

#### 运行其他 rosbag 数据

注意, rosbag 文件需要发布一个主题为 /velodyne_points, 消息类型为 sensor_msgs/PointCloud2 的点云序列. 你也可以通过下一节介绍的方式, 从 KITTI 数据集的原始激光雷达点云数据中, 构建 rosbag 文件. KITTI 数据集可以从 <a href="http://www.cvlibs.net/datasets/kitti/eval_odometry.php">KITTI 官网</a>下载

```shell
# yourpath 代表要运行的 rosbag 的绝对文件路径
roslaunch simple_frame mapping.launch rosbag:=/yourpath
```

#### 从 KITTI 数据集中构建 rosbag 数据包

```shell
# 创建输出文件夹用于保存 rosbag
sudo mkdir -p /KITTI/bag
sudo chmod a+wx /KITTI/bag

# 使用 A-LOAM 提供的 KITTI_Helper 工具进行转化
roslaunch aloam_velodyne kitti_helper_simple.launch dataset_floder:=/KITTI/ sequence_number:=00 output_bag_file:=/KITTI/bag/kitti00.bag
```

在运行此命令前要保证数据集的文件结构如下：

```
/(根目录)
|__KITTI(数据集目录)
	|——velodyne
	|	|__sequences
	|		|——00
	|		|	|——calib.txt	(雷达标定参数文件)
	|		|	|——times.txt	(点云文件时间戳)
	|		|	|__velodyne
	|		|		|——000000.bin (LiDAR点云文件)
	|		|		|——000001.bin
	|		|		|__.......bin
	|		|——01
	|		|——02
	|		|__......
	|__bag (数据包输出文件夹)
```

### 项目结构

#### 文件代码结构

项目文件的主要构成部分如下：

```
LidarPointCloudReconstruction
├── data 
│   └── nsh_indoor_outdoor.bag
├── aloam 
├── slam_trans
├── frame_reconstruction
├── frames_fusion
├── poisson_reconstruction
├── img
├── ReadMe.md
└── environment.yml

```

- **data**. 存储有已经整合好的 rosbag 数据包, 作为展示示例.
- **aloam**. SLAM&点云配准和位姿估计包, 包含有转化 KITTI 数据集的节点, 以及将多帧输入点云配准并且估计激光雷达位姿的节点等. 参见 <a href="https://github.com/HKUST-Aerial-Robotics/A-LOAM">aloam</a>.
- **slam_trans**. SLAM输出对接包, 将 aloam_velodyne 节点输出的点云以及位姿消息转化为后续节点可用的形式, 并且完成点云与位姿的同步工作.
- **frame_reconstruction**. 单帧重建包, **主要算法包**之一, 将单帧点云实时重建为三角形网格, 采用的重建方法是基于可见性的表面重建. 利用重建出的三角形网格可以估计出单帧点云的法向量, 并且可以根据视线与平面的夹角设置点的置信度, 为更加精确的重建作准备.
- **frames_fusion**. 多帧融合包, **主要算法包**之一, 将单帧重建后获得的带有法向量的点云进行融合和优化, 并使用优化后的点云生成类 TSDF 场, 最后使用 Marching Cube 方法生成三角形网格.
- **poisson_reconstruction**. 离线泊松重建包. 将优化后的带有法向量的点云进行离线重建, 可以得到更加精细的重建结果. 算法思路和部分代码采用 <a href="https://github.com/PRBonn/puma">puma</a>.
- img. 存储 ReadMe.md 说明文档中用到的图片.
- ReadMe.md. 说明文件.
- environment.yml. anaconda 环境配置文件, 用于配置 poisson_reconstruction 包所需的python 环境.

#### ROS 节点结构

项目运行过程中, 可以使用以下命令查看各个节点之间的输入输出关系：

```shell
rosrun rqt_graph rqt_graph
```

下图给出在运行示例数据 nsh_indoor_outdoor.bag 时的节点关系图：

![img](img/sendpix1.jpg)

![img](img/sendpix2.jpg)

其中, 椭圆代表程序节点, 矩形代表主题, 不同节点通过在主题中传递消息. 根据该图可以看出, 整个程序以流水线的形式对点云进行重建. 

1. 从 rosbag 节点出发，播放原始的激光雷达点云到主题 /velodyne_point 上.
2. 接着 aloam 的第一个节点 ascanRegistration 从主题 /velodyne_point 中接收到原始的点云信息, 然后通过其三个节点 (/ascanRegistration, /alaserOdometry, /alaserMapping) 对点云进行处理, 并输出配准后的点云到主题 /velodyne_cloud_registered 上, 输出 LiDAR 位姿信息到主题 /aft_mapped_to_init 上. 
3. 接着 /slam_transfer 将 aloam 输出的点云信息和位姿信息转换到全局坐标系之下, 并重新输出到主题 /slam_points 和 /slam_odom 上.
4. 节点 /frame_reconstruction 进行单帧重建, 将带有法向的点云发布到 /frame_cloudnormals 主题, 将重建出来的初步的网格发布到 /frame_meshs 上, 以用于在虚拟环境下的展示 (虚拟环境下的白色网格).
5. 节点 /frames_fusion 进行多帧重建, 将融合后的点云发布到 /processed_clouds 主题, 将进一步重建的网格结果发布到 /surrounding_meshs 上, 以用于在虚拟环境下的展示 (虚拟环境下的黄色网格).
6. 节点 /n__poisson_reconstruction 进行离线泊松重建, 将最终的重建结果发布到 /poisson_mesh 主题, 以用于在虚拟环境下的展示 (虚拟环境下的绿色网格).

### 运行界面说明

本节对界面的主要部分进行介绍. 本项目采用 ROS 中的 RViz 工具实现可视化功能, 从界面中可以实时观测重建结果. 界面总体分为三栏, 从左到右依次为：场景树, 场景窗口, 摄像机设置. 在场景树中, 有四个文件夹, 作为主要的显示元素. 

![img](img/sendpix3.jpg)

首先介绍 SLAM 文件夹, 该文件夹负责显示与 alom 结果相关的数据, 包括 odomPath (未经矫正的 LiDAR 位姿), mappingPath (矫正后的 LiDAR 位姿, 在场景窗口中显示为绿色线条), allMapCloud (配准后的点云, 在场景窗口中显示为白色点云), 以及 outputPoints (当前帧的点云, 在场景中显示为彩虹色点云).

![img](img/sendpix4.jpg)

下面三个文件夹 SingleFrame, FramesFusion, Poisson 分别显示了实时单帧重建, 多帧重建, 泊松离线重建的网格结果. 在场景窗口中分别显示为白色, 黄色, 绿色的网格模型. 值得注意的是, 在重建的过程中, 泊松重建速度很慢, 需要等待较长时间才能重建出来, 因此落后于 LiDAR 当前帧的位置; 而实时单帧重建和多帧重建速度较快, 生成的网格结果会一直跟随 LiDAR 移动.

## 运行结果说明及分析

### 单帧重建结果

#### 重建网格质量

下图为程序运行时的截图, 只显示了单帧重建的结果, 以及单帧点云和雷达运动路径. ![img](img/sendpix11.jpg)

从图中可以看出, 一方面单帧重建算法对于场景中的地面和墙壁等物体的重建效果较好, 另一方面, 对于树木这类的小而复杂的物体重建效果较差. 究其根本, 是因为单帧点云较为稀疏, 对于大面积的规则平面有较好的表示能力, 但对于一些分辨率需求较高的物体, 单帧点云就会遗漏很多信息, 无法很好表示. 想要恢复出精细的场景, 仅仅依靠单帧点云是不现实的, 而需要通过将多帧点云进行叠加融合补全缺失的信息. 这时单帧重建的作用则变成了——为多帧点云融合提供较为准确的点云法向.

另外, 在图中的左下角和右侧, 存在着大量杂而零散的错误面片. 这些面片是由重建算法的特性导致的. 基于可见性的算法倾向于计算一个水密的表面, 因此对于深度断层的场景 (例如墙前面有几棵树, 或者墙前面有个广告牌等情况), 基于可见性的算法会将前景 (树或者广告牌) 和背景 (墙面) 通过面片连接到一起. 图示结果是经过处理的重建网格, 但是通过目前的处理方法, 网格中仍然残留有较多伪面. 具体的重建和处理方法详见**基于可见性的重建**. 

#### 算法时间性能

> 本项目在 Intel® Xeon(R) CPU E5-2620 v4 @ 2.10GHz × 32GB 硬件环境运行

基于可见性的单帧重建算法属于实时算法, 因此算法的执行时间是衡量重建算法性能的一项重要指标. 下图是程序运行时控制台输出的结果, 可以看出对于数据量在13,000左右的单帧点云, 算法的**重建时间约为 100ms, 频率为 10Hz**, 满足实时重建的需求.![img](img/sendpix5.jpg)![img](img/sendpix6.jpg)

对于数据量在100,000左右的单帧点云来说, 重建时间接近1s,  最坏的时间甚至在2s以上, 完全不满足实时性的需求. 在这种情况下, 由于重建速度慢, 导致重建过程中有大量帧被漏掉 (接收到1325帧, 而实际重建出来只有467帧), 这是不被允许的.![img](img/sendpix7.jpg)![img](img/sendpix8.jpg)

对于这种情况, 可以采用**点云下采样**的方式, 将单帧点云的数据量从100,000量级, 采样降到10,000量级, 这样就可以进行实时重建, 并且经过观察, 重建质量基本不会受到影响.![img](img/sendpix9.jpg)![img](img/sendpix10.jpg)

### 多帧融合结果

#### 重建网格质量

通过单帧点云重建得到的带有法向量的点云, 然后将多帧带有法向量的点云进行融合, 理论上可以获得更高质量的重建结果. 多帧重建主要使用类似于 TSDF 的方式, 使用点云代替射线更新空间中的体素, 具体的重建方法参见**基于 TSDF 的重建**. 在实际重建过程中, 为了控制算法的复杂度, 将多帧融合的点云范围限定在直径为 $d$ 的球体内部 (可以在 `/frames_fusion/launch/fusion.launch`文件中找到对应的参数`voxel_total_size`). 整体重建效果如图所示.![img](img/sendpix13.jpg)![img](img/sendpix14.jpg)

从图中可以看到, 多帧点云重建对于墙壁接近地面的部分重建结果良好, 而对于地面、墙壁远离地面的部分、树木等重建效果较差. 下面对这三种物体一一进行分析：

- 地面. 多帧融合重建出来的地面出现大量空洞, 一方面是因为点云并非足够稠密, 即使是多帧点云叠加也会有空隙存在. 另一方面, 单帧重建点云所提供的法向量在地面处有错误, 通过导出直接叠加后的点云可以观察到这一点, 如图所示. ![img](img/pix0.jpg)

  这张图是从地底向上观察的视角, 点为亮黄色说明点的法向朝向视角, 因此亮黄色的点代表法向有错误的点 (因为地面上的点法向应该从地面向上, 而不是从地面向下). 之所以会有这样的错误, 目前猜测是因为在单帧重建的网格中存在较多干扰面片, 如图所示.![img](img/sendpix12.jpg)

  从图中可以看到, 最接近雷达的一圈红色点云, 它们之间会构成面片, 这种面片通常会提供错误的法向量, 虽然每一帧的法向量错误的点很少 (只有一圈) , 但是多帧叠加起来错误点仍然数量可观. 将点云法向量进行融合, 可以缓解错误法向量带来的影响, 如图所示. ![img](img/pix1.jpg)

  即使使用多帧点云融合法向, 但由于错误点较多, 仍然无法消除全部的错误法向. 为了消除完全消除错误法向, 需要从源头做起, 目前此问题尚未得到解决, 以下提出两种解决设想方案：1. 删除靠近 LiDAR 内圈的点. 这种方案简单直接, 但有可能会降低多帧点云的稠密度. 2. 删除错误的面片. 没有错误的面片就没有错误的法向, 需要寻找一种高效的删除此类错误面片的方法 (至少不影响实时性能).

- 墙壁远离地面的部分. 对于这部分的重建, 受 LiDAR 扫描高度的限制, 对于场景中较高且较远的部分, 扫描得到的点云会比较稀疏, 甚至根本无法扫描到. 因此, 将更为稀疏的点云进行多帧融合, 会得到不那么稠密的结果, 重建出来的网格也是零散的三角形面片. 

  *补充：*对于墙壁和地面来说, 基于可见性的重建方法表现效果良好, 因此还有另外一种可能的方法用于补足空洞. 即使用多帧叠加后的点云, 再次进行基于可见性的重建, 然后在重建出的网格上采样点云, 以填补地面和墙壁上的空洞.

- 树木. 对于树木等具有丰富细节的物体的重建, 受到 LiDAR 扫描分辨率的限制. LiDAR 扫描到的点较为稀疏, 即使是多帧点云, 也无法很好表示树木中的树叶、枝干等高分辨率部分. 另外, 使用类似 TSDF 的方法进行重建, 会使得重建结果受到体素分辨率的限制, 使得树的重建结果变成杂乱而零散的三角面片. 

#### 算法时间性能

对于多帧重建, 对实时性能的要求较低, 目前可以达到每次重建平均 400ms 左右的效果. 下图是程序运行时的部分控制台输出.![img](img/sendpix15.jpg)![img](img/sendpix16.jpg)

### 泊松重建结果

泊松重建是一种离线重建算法, 以融合优化过后的法向为输入进行重建, 重建算法详见**基于泊松的重建**. 本项目中采用的泊松重建算法, 设定八叉树的最大深度 $depth=10$, 可以在`possion_reconstruction/launch/poisson.launch`中找到其对应参数`poisson_depth`.泊松重建算法在程序运行中的结果如下图所示. ![img](img/pix3.jpg)

从较远的视角观察全局可以发现, 在远处点云稀疏的地方, 重建结果有多片小的表面. 这是因为原始的泊松重建算法重建出一个水密模型, 这个模型存在许多多余的面片. 为了剔除这些面片, 算法将周围点云较少的面片剔除, 因此大部分点云稀疏的地方的网格被剔除, 但远处仍有一些地方, 其点云比较稠密, 导致残留了许多小的表面. 

![img](img/sendpix18.jpg)

从距离模型较近的视角观察, 可以看出泊松重建对于墙面和地面有着更好的重建效果, 并且对于路边的汽车、 告示牌、墙上的窗户等重建出的模型都初具雏形.  对于复杂的树木、较细的路灯杆还不能完成较好的重建. 

![img](img/pix6.jpg)

如果采用较深的八叉树最大深度 $depth=12$, 则会得到更加精细的结果, 如汽车、路牌等, 但是也会在点云较少的地方出现孔洞 (这个孔洞仍然是面片剔除造成的), 并且地面和墙壁会因为点云法向量的抖动和点云配准的误差出现坑和“划痕“. 如果在泊松重建之前进行点云滤波等操作或许会减弱这一影响.

## 算法思路及各种尝试

### SLAM 包的选择

本项目尝试了两个算法包 LOAM 和 A-LOAM, 都是采用了只通过 LiDAR 的点云进行位姿估计和点云配准的算法, 具体的算法原理参见 [LOAM: Lidar odometry and mapping in real-time](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf). 本项目对两个算法包进行测试, 发现 A-LOAM 拥有更好的实时性能, 以下两张图片依次是运行 LOAM 和 A-LOAM 时控制台的输出 (输入为 nsh_indoor_outdoor.bag). 

![img](img/pix4.jpg)

![img](img/pix5.jpg)

对于同一输入点云序列, 使用 A-LOAM 接收到的总帧数较多, 并且单帧重建的时间较短, 因此选择 A-LOAM 作为 SLAM 包更为合适.

### 基于可见性的重建

基于可见性的重建基本算法可参考 [HPR](), 其基本思路是使用函数将点云推向远离视点的位置, 然后构建点云和视点的凸包, 在凸包上的点对应的原来的点云上的点即为可见点. 由于凸包上的每个点都可以在原来的点云中寻找到对应点, 那么凸包上的每一个三角形面片就可以通过三对对应点关系映射成一个新的面, 将凸包中所有的面进行映射得到的新的表面, 就是最后的重建结果. 

基于可见性的重建算法在以往都用在水密物体的重建过程中, 将这个重建算法迁移到 LiDAR点云重建需要注意如下问题：

1. 需要剔除顶部天空的错误面片. 在开放街道场景中, 本不该在天空中出现封闭面片, 这类面片是错误的, 如图所示. ![img](img/Picture2.png) ![img](img/Picture1.png)

   为了解释顶部错误面片生成的原理, 我们将经过 HPR 函数变换后的点云进行了输出, 结果如下. ![img](img/Picture3.png)

   可以想象, 经过变换后的点云, 对其构建凸包, 其实会生成鼓的形状. 然而, 鼓顶部的表面就对应着天空中错误的面片. 要想剔除这些面片, 可以采用切分的方式, 将整个点云沿着视点中心按角度平均分为12份, 然后再进行重建, 这样就可以避免生成顶部的错误面片. 对应参数`sector_num`可在`frame_reconstruction/launch/reconstruction.launch `文件中找到.

   另外, 切分场景的方法还有利于将算法转化为并行算法而加快计算速度, 进而提高算法的实时性能. 事实上, 我们也曾对算法并行化进行尝试, 然而, 由于 `pcl`库中的凸包构建算法 (重建过程中耗时最长的子算法) 并非线程安全算法, 实际运行时需要添加互斥锁, 并杏化的程序反而拖慢了重建速度. 因此需要另寻他法, 一方面可以寻找新的线程安全的库函数进行重建, 另一方面也可以考虑使用多进程的方式实现进程级并行来避免线程安全问题. 

2. 需要剔除深度断层产生的粘连面片.  基于可见性的算法会将前景和背景面片粘连, 生成粘连面片, 如图所示.![img](img/Picture4.png)

   对于这种面片, 可以采用视线判断的方式解决, 从视点中心向面片发出射线, 如果射线与面片法向量的夹角大于一定阈值, 就将其剔除. 以这种方式可以剔除大部分粘连面片, 结果如图所示. ![img](img/Picture5.png)

   然而在树等复杂结构的附近仍然能够看到一些残留的粘连面片. 由于基于可见性的算法重建出来的网格不可避免地出现无法剔除的伪面, 因此才需要下一步基于 TSDF 的重建进行进一步优化.

### 基于 TSDF 的重建

基于 TSDF 的重建原本用于深度相机下的重建, 通过深度图向场景投射射线来更新隐式场, 进而完成重建过程, [KinectFusion](https://dl.acm.org/doi/pdf/10.1145/2047196.2047270) 是经典的使用 TSDF 进行重建的方法. 我们希望将这种方法迁移到 LiDAR 点云的重建当中, 在这个过程中最大的问题就是相比深度相机来说, LiDAR 生成的点云过于稀疏, 这对 TSDF 场的更新造成了极大的挑战.

我们提出了根据带有法向的点云更新 TSDF 场的方法, 具体的重建步骤如下：

1. 首先划定重建范围并确定体素大小, 因为基于体素的重建方法不能承受过大范围和过高分辨率的重建任务. 项目中重建范围和体素大小的默认值设置为 30.0m 和 0.2m, 对应的参数`voxel_total_size`和`voxel_cube_size`可以在文件`frames_fusion/launch/fusion.launch`中找到. 
2. 对于场景中的每个体素, 寻找在体素中的点云, 并将它们聚合到体素之中. 这样, 对于每一个内部存在点的体素, 都有一个融合后的带有法向的点在体素内部. 在点云融合的过程中, 可以使用权重作为指导, 如视点到点的射线与点法向夹角的余弦值.
3. 带有法向的点可以看作是一个小型的面片, 因此可以使用这个面片来更新体素的每个角点的 TSDF 值. 
4. 在所有的体素更新之后, 即可使用 Marching Cubes 算法重建出网格. 在这一步中, 值的注意的是, 因为只有部分包含点的体素被更新, 因此也只有被更新的体素才能运行 Marching Cubes 算法生成网格. 在项目中的实现方法是, 使用数组记录被更新的体素然后将其余的体素过滤掉, Marching Cube 计算时需要关闭其加速算法以保证重建完整. 

### 基于泊松的重建

泊松重建是经典的离线重建算法, 通过法向点云构建梯度场, 接着通过求解泊松方程得到指示函数, 最终使用 Marching Cubes 重建出网格模型. [Poisson Surface Reconstruction for LiDAR Odometry and Mapping](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2021icra.pdf) 是基于泊松算法对 LiDAR 点云的工作, 在本项目中采用的泊松重建算法的核心思路也主要来自于此工作. 这项工作中还提到泊松算法对于重建 LiDAR 开放场景的适配方法, 通过剔除没有点云支持平面的方式, 来去除冗余的面片, 进而提高重建结果的准确率.

泊松重建对于点云的法向较为敏感, 准确的法线向量可以使泊松算法得出更准确的重建结果. [Poisson Surface Reconstruction for LiDAR Odometry and Mapping](http://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/vizzo2021icra.pdf) 中采用简单的相邻点向量外积的方式得到点云的法向量, 通过这种方式得到的法向量往往不够准确. 本项目将多帧点云融合, 期望得到更加精确的法向量. 