---
layout: post
title: '3 机器人导航 global_planner'
date: 2019-08-29
author: Green
color: rgb(255,210,32)
cover: '../2019/08/29/cover.jpg'
tags: navigation
---

# global planner

&#8195;&#8195;在对move_base中的全局规划进行介绍之前，我们先来回顾一下Navigation的框架，可以看出位于红框中的global_planner是move_base节点的一部分，输入是目标位置以及global_costmap的信息，输出是全局的规划路径，作为local_planner的输入。

<figure align="center">
  <img src="框架.png" width="75%" height="75%"/>
</figure>

&#8195;&#8195;在ROS的导航中，首先会通过全局路径规划，计算出机器人到目标位置的全局路线。全局路径规划一般是由navfn或global_planner插件实现的。

- **navfn**
通过Dijkstra最优路径的算法，计算costmap上的最小花费路径，作为机器人的全局路线。在算法上还有A*算法。

<figure align="center">
  <img src="navfn.png" width="75%" height="75%"/>
</figure>

- **global_planner**
根据给定的目标位置进行总体路径的规划；这个package为导航提供了一种快速，内插值的全局规划器， 继承了nav_core包中nav_core::BaseGlobalPlanner接口，该实现相比navfn使用更加灵活。

<figure align="center">
  <img src="g_b.png" width="75%" height="75%"/>
</figure>

## 路径规划算法

&#8195;&#8195;路径规划是指的是机器人的最优路径规划问题，即依据某个或某些优化准则（如工作代价最小、行走路径最短、行走时间最短等），在工作空间中找到一个从起始位置到目标位置能避开障碍物的最优路径。

&#8195;&#8195;路径规划的方法主要可以分为以下五类，global_planner主要用到的就是第二类基于节点的方法。

<figure align="center">
  <img src="路径规划算法概况.png" width="75%" height="75%"/>
</figure>

### Dijkstra算法

&#8195;&#8195;迪杰斯特拉（Dijkstra）算法是典型的用来解决最短路径的算法，也是很多教程中的范例，由荷兰计算机科学家狄克斯特拉于1959年提出，用来求得从起始点到其他所有点最短路径。该算法采用了贪心的思想。

&#8195;&#8195;下面结合实例来看一下。

<figure align="center">
  <img src="d1.png" width="75%" height="75%"/>
</figure>

```
s是start顶点，t是terminal顶点，要求出s到t的最短路径。
```

<figure align="center">
  <img src="d2.png" width="75%" height="75%"/>
</figure>

```
首先我们初始化了两个集合，S集合是已计算出最短路径的顶点集合，PQ是未计算出最短路径的顶点的集合。
顶点附近的绿色数字表示从源节点到当前节点的距离，若二者没有直接相连，则为无穷大（∞）。
```
<figure align="center">
  <img src="d3.png" width="75%" height="75%"/>
</figure>

```
从PQ的所有顶点中，选择距离值最小的顶点放入集合S中。
```
<figure align="center">
  <img src="d4.png" width="75%" height="75%"/>
</figure>

```
首先将起始点s放入集合S中，然后将距离进行更新，可见图中2、6、7顶点的距离值发生了变化。
```

<figure align="center">
  <img src="d5.png" width="75%" height="75%"/>
</figure>

```
此时PQ集合中距离值最小的顶点是2号顶点。
```

<figure align="center">
  <img src="d6.png" width="75%" height="75%"/>
</figure>

```
此时将2号顶点放入集合S中，同时记录其父顶点为S。
```

<figure align="center">
  <img src="d7.png" width="75%" height="75%"/>
</figure>

```
然后将距离值进行更新。
```

<figure align="center">
  <img src="d8.png" width="75%" height="75%"/>
</figure>

```
此时PQ中6号顶点距离值最小。
```

<figure align="center">
  <img src="d9.png" width="75%" height="75%"/>
</figure>

```
将6号顶点加入集合S，同时，PQ中的5号和3号顶点距离值进行了更新。
```

<figure align="center">
  <img src="d10.png" width="75%" height="75%"/>
</figure>

```
此时，PQ中7号顶点的距离值最小。
```

<figure align="center">
  <img src="d11.png" width="75%" height="75%"/>
</figure>

```
7号顶点加入集合S，同时5号和t顶点距离值更新。
```

<figure align="center">
  <img src="d12.png" width="75%" height="75%"/>
</figure>

```
此时，PQ中3号顶点的距离值最小。
```

<figure align="center">
  <img src="d13.png" width="75%" height="75%"/>
</figure>

```
3号顶点加入集合S，同时5号和t顶点距离值更新。
```

<figure align="center">
  <img src="d14.png" width="75%" height="75%"/>
</figure>

```
此时，PQ中5号顶点的距离值最小。
```

<figure align="center">
  <img src="d15.png" width="75%" height="75%"/>
</figure>

```
5号顶点加入集合S，同时4号和t顶点距离值更新。
```

<figure align="center">
  <img src="d16.png" width="75%" height="75%"/>
</figure>

```
此时，PQ中4号顶点的距离值最小。
```

<figure align="center">
  <img src="d17.png" width="75%" height="75%"/>
</figure>

```
4号顶点加入集合S，此时t的距离值由于未减小，因此没有发生变化。
```

<figure align="center">
  <img src="d18.png" width="75%" height="75%"/>
</figure>

```
此时仅剩最后一个t顶点。
```

<figure align="center">
  <img src="d19.png" width="75%" height="75%"/>
</figure>

```
t顶点加入集合S。
```

<figure align="center">
  <img src="d20.png" width="75%" height="75%"/>
</figure>

```
实际上，我们可以得到起始点s到各个顶点的最短距离：2(9),3(32),4(45),5(34),6(14),7(15),t(50) 。
从起始点s到目标点t的最短路径为s->6->3->5->t。
```
&#8195;&#8195;总结一下**算法流程**：

1. 初始化时，S只含有源节点s；
2. 从PQ中选取一个距离s最小的顶点k加入S中（该选定的距离就是s到k的最短路径长度）；
3. 以k为新考虑的中间点，修改PQ中各顶点的距离；若从源节点s到顶点u的距离（经过顶点k）比原来距离（不经过顶点k）短，则修改顶点u的距离值，修改后的距离值是顶点k的距离加上k到u的距离；
4. 重复步骤2和3，直到所有顶点都包含在S中。

### A*算法

&#8195;&#8195;同样地，我们结合实例来看一下A*算法是如何实现的。

<figure align="center">
  <img src="a1.png" width="50%" height="50%"/>
</figure>

```
绿色方格是我们的起始点，红色方格是我们的目标点，蓝色表示障碍物，我们需要寻找起始点到目标点的
路径。
```

<figure align="center">
  <img src="a2.png" width="50%" height="50%"/>
</figure>

```
首先，将图中的搜索区域用二维数组表示，数组的每一项代表一个格子，它的状态分为可走 (walkalbe) 
和不可走 (unwalkable) 。

我们还需要初始化Open List和Close List，Open List用于存放路径规划过程中待检测的格子，而
Close List用于存放已检测过的格子。我们先将起始点A(1,2)放入Closed List，然后把相邻的可走的 
方格加入到 Open List 中，然后把起点 A 设置为这些方格的父节点，方便路径追踪。

我们引入一个计算公式：F = G + H。其中G代表的是从起始节点沿着已生成的路径到指定待检测格子的
移动开销。H指待测格子到目标节点的估计移动开销，也称为启发项。假设横向或纵向移动的开销G是10，
沿对角线移动的开销是14，H是当前方格到目标方格的横向或纵向移动开销之和。

因为Open List中方格(2,2)的F值最小，因此将(2,2)加入Closed List并作为当前要处理的节点。
```


<figure align="center">
  <img src="a3.png" width="50%" height="50%"/>
</figure>

```
(2,2)的8个相邻方格中，有3个是障碍物，1个是在Close List中，忽略这几个方格。

剩下的4个方格都是在Open List中的方格，因此检查经由(2,2)到这4个方格的路径是否更好，用G来衡量，
比如原来从(1,2)到(2,3)的G为14，若从(1,2)先经过(2,2)再到(2,3)，则G变为20，显然还不如原来的路
径，对于其它3个方格亦是如此。
```

<figure align="center">
  <img src="a4.png" width="50%" height="50%"/>
</figure>

```
将(2,1)加入Close List，其相邻的8个方格中，2个障碍物的方格和障碍物下的方格(3,0)认为不可达，
(1,2)和(2,2)在Close List中不做处理，Open List中的(1,1)没有优化原来的路径。

(1,0)和(2,0)加入Open List，并将父节点设置为(2,1)，分别计算F、G、H值。
```

<figure align="center">
  <img src="a5.png" width="50%" height="50%"/>
</figure>

```
以此类推，直到目标方格B加入Open List。
```

<figure align="center">
  <img src="a6.png" width="50%" height="50%"/>
</figure>

```
从目标方格B开始沿着父节点移动到起始方格A，就是求得的路径：
(5,2)->(5,1)->(4,0)->(3,0)->(2,0)->(2,1)->(1,2)。
```
### 对比

- **Dijkstra 算法**需要载入全部数据进行全局遍历，确保运算结果一定是最短路径。
- **A*算法**的核心思想在于设置的代价函数F=H+G，来寻找距离start和goal代价距离和F最少的点。

## 参考资料
[1] https://blog.csdn.net/heroacool/article/details/51014824

[2] https://blog.csdn.net/weixin_44489823/article/details/89382502