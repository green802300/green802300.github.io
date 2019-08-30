---
layout: post
title: '4 机器人导航 local_planner'
date: 2019-08-30
author: Green
color: rgb(255,210,32)
cover: '../2019/08/30/cover.jpg'
tags: navigation
---

# local planner

&#8195;&#8195;我们先来回顾一下Navigation的框架，可以看出位于红框中的local_planner是move_base节点的一部分，输入包括来自global_planner的全局的规划路径、里程计信息以及local_costmap的信息，输出是速度控制指令，作为控制器的输入。

<figure align="center">
  <img src="框架.png" width="75%" height="75%"/>
</figure>

## base_local_planner

&#8195;&#8195;局部规划一般是用base_local_planner包实现的，该包使用**Trajectory Rollout**和**Dynamic Window approaches(DWA)**算法，根据地图数据，通过算法搜索到达目标的多条路经，利用一些评价标准（是否会撞击障碍物，所需要的时间等等）选取最优的路径，并且计算所需要的实时速度和角度。

### DWA算法

&#8195;&#8195;这里有一张DWA算法的示意图，蓝色的方块表示机器人，红色表示障碍物，机器人前方的多条黑色虚线表示备选的路线（可以设置参数来调整虚线的数量，也可以设置参数来调整虚线的长度，虚线越多越长对计算资源的消耗越多，但也会得到更为精确的结果），DWA算法会对每条路线进行评分，最终选择得分最高的路线，而会导致碰撞发生的路线会被排除。

<figure align="center">
  <img src="DWA.png" width="50%" height="50%"/>
</figure>


&#8195;&#8195;Trajectory Rollout和DWA算法的主要思路如下：

1. 采样机器人当前的状态（dx,dy,dtheta）； 
2. 针对每个采样的速度，计算机器人以该速度行驶一段时间后的状态，得出一条行驶的路线； 
3. 利用一些评价标准为多条路线打分；
4. 根据打分，选择最优路径；
5. 重复上面过程。

&#8195;&#8195;Trajectory Rollout和Dynamic Window Approach (DWA)两种方法，理论上来说分别对应base_local_planner和dwa_local_planner两个包，但其实dwa的大部分代码都放在了base_local_planner包里面。

### 类继承关系

&#8195;&#8195;base_local_planner类继承图如图所示。

<figure align="center">
  <img src="类继承图.png" width="80%" height="80%"/>
</figure>

- **TrajectoryPlanners**（红框标出）实现了DWA 和Trajectory Rollout算法。
- **TrajectorySampleGenerator**产生一系列轨迹
- 然后**TrajectoryCostFunction**遍历轨迹打分
- **TrajectorySearch**找到最好的轨迹拿来给小车导航
- 由于小车不是一个质点，**WorldModel**会检查小车有没有碰到障碍物

## teb_local_planner

&#8195;&#8195;teb_local_planner也是local planner的一个插件。teb即Timed Elastic Band，基本思路是根据轨迹执行时间、避障、动力学约束（如最大速度和最大加速度的限制）等来优化机器人的轨迹。

### TEB算法

&#8195;&#8195;对于二维路径的描述，有一个有趣的方法，叫做Elatic Band（橡皮筋）。简而言之，就是连接起始、目标点，并让这个路径可以变形，变形的条件就是将所有约束当做橡皮筋的外力。

&#8195;&#8195;经典的“elastic band”由n个的机器人位姿组成的序列描述$X_i=(x_i,y_i,\beta_i)^T$，其中$x_i$，$y_i$是机器人位置，$\beta_i$被定义为全局中机器人的方向。

$$
Q=\{X_i\}_{i=0...n}
$$

&#8195;&#8195;为了显示轨迹的运动学信息，我们在点与点之间定义运动时间Time。于是，这个方法就叫做**Timed-Elastic-Band**。

&#8195;&#8195;TEB在原有的基础上加入了时间间隔序列：

$$
\tau=\{\Delta{T_i}\}_{i=0...n-1}
$$

&#8195;&#8195;每个时间间隔表示机器人需要从当前位姿转换到序列$Q$中的下一个位姿所需要的时间。

<figure align="center">
  <img src="TEB.png" width="40%" height="40%"/>
</figure>

&#8195;&#8195;因此，TEB由上述两个序列组成。

$$
B:=(Q,\tau)
$$

&#8195;&#8195;TEB算法的**目标函数**主要包括：

1. **跟随全局路径+避开障碍物**。跟随全局路径使得elastic bands靠近全局路径，而避障约束使得elastic bands远离障碍物。
2. **速度和加速度限制**。这一条限制即简单的不等式约束。
3. **运动学限制**。一般情况下机器人在平面运动只有两个自由度，其只能以朝向的方向直线运动或旋转。这种运动学约束使得机器人以有若干弧段组成的平滑的轨迹运动。对于某些机器人，会有最小转弯半径的约束（因为不能原地旋转）。
4. **其他限制**。比如使时间间隔序列的平方和最小。

&#8195;&#8195;我们的目标是进行局部的路径规划：

$$
f(B)=\sum_{k}\gamma_kf_k(B)
$$

$$
B^*=\arg\min_Bf(B)
$$

&#8195;&#8195;$f(B)$是全局目标函数，它是考虑各种约束的目标函数的加权和，$B^*$是被优化的TEB序列结果，也就是我们需要的局部规划的路径。

&#8195;&#8195;这是一个复杂的多目标优化问题，虽然看似复杂，但是这就是一个**bundle adjustment**问题。我们可以将它描述成图，然后用图优化进行求解。

<figure align="center">
  <img src="T1.png" width="60%" height="60%"/>
</figure>

```
如图，这个图的节点(vertexs)是橡皮筋的状态（机器人姿态+时间）。
```

<figure align="center">
  <img src="T2.png" width="60%" height="60%"/>
</figure>

```
图的边edges是我们自己定义的目标函数，这张图加入了速度约束。
```

<figure align="center">
  <img src="T3.png" width="60%" height="60%"/>
</figure>

```
这张图加入了加速度约束。
```

<figure align="center">
  <img src="T4.png" width="60%" height="60%"/>
</figure>

```
这张图加入了障碍物约束。

求解的框架，可以使用g2o（A General Framework for Graph Optimization）。当然，节点和边的类型
需要我们自己使用g2o中的模板定义。
```
## 参考资料

[1] http://wiki.ros.org/base_local_planner

[2] https://www.leiphone.com/news/201612/0TCtaBOIcFOIBN69.html