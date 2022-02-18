# 布料模拟

## Summary

传统的布料模拟使用的是基于力的做法，其基本步骤为：
* 根据位置约束计算出质点受力
* 根据受力计算出加速度
* 利用加速度迭代速度
* 利用速度迭代位置

但这种模拟方式需要较小的迭代步长，计算量较大，所以我使用了Position Base Dynamics算法，同时使用了unity的Job System进行并行计算，该算法的优点是可以使用较大的迭代步长，并且Job System配合Burst编译，性能非常可观。

![](https://s2.loli.net/2022/01/12/LN5cxovImlHegJj.gif)

## PBD(Position Base Dynamics) Process

PBD算法的流程如下

![](https://pic3.zhimg.com/80/v2-cfa71169bcab6c7cf3c6c7f8041e61ee_720w.jpg)

图中x为当前质点位置,p为预测质点位置。可以看到:

* 算法在每一帧开始时，会根据外力和阻尼先于计算一个位置(p)
* 根据当前位置(x)和预测位置(p)，去进行碰撞检测，如果发现有碰撞，就生成一个碰撞约束
* 迭代求解所有约束，最终会得到新预测位置p
* 利用p - x 来计算速度v，并将p赋值给x
* 最后再计算一遍速度(处理碰撞反弹之类速度变换)

### 外部力和约束的定义

像重力、风、可以视为外部力，但是像质点之间的相互作用我们通常视为内部约束，而非力。同样外部物体对质点的碰撞，也视为碰撞约束而不是外部力。

### 质点及布料模型

在物理上，我们通常可以将布料视作由一个个质点组成的结构。质点之间满足一定的约束条件，来模拟布料内部的作用力，如下图所示:

![](https://pic2.zhimg.com/80/v2-fae7d8322e3906695060a236e7270395_720w.jpg)

在传统的基于力的模拟中，质点之间的约束通常是用胡克定律以弹簧振子的形式去模拟。但是在基于位置的方案中，我们将会采用另外一种约束公式。

质点与布料模型顶点一一对应，然后让三角形的边作为顶点之间的约束。 好处是无需额外的骨骼绑定工作，我们可以针对满足一定的拓扑条件的任意Mesh进行布料模拟(1:一条边只被两个三角形共享 2:不封闭)，坏处是如果模型顶点较多，将比较耗费性能。

### 质量生成算法

通常来说，我们可以给一块布指定其密度(kg/m^2)。 这样的话，我们就可以根据Mesh的面积来计算质量。具体来说就是:

* 遍历Mesh的所有三角面
* 计算每个三角面的面积，乘以密度得到质量
* 将质量三等份，分别累加给每个顶点

```c#
_masses = new NativeList<float>(this.vertexCount,Allocator.Persistent);
_masses.Resize(this.vertexCount,NativeArrayOptions.ClearMemory);
for(var i = 0; i < indices.Length / 3; i++){
    var offset = i * 3;
    var i0 = indices[offset];
    var i1 = indices[offset + 1];
    var i2 = indices[offset + 2];
    var v0 = vertices[i0];
    var v1 = vertices[i1];
    var v2 = vertices[i2];
    var area = IntersectUtil.GetArea(v0,v1,v2);
    var m = area * _setting.density;
    var m3 = m / 3;
    _masses[i0] += m3;
    _masses[i1] += m3;
    _masses[i2] += m3;
}
```

### 预测位置计算

为了使用JobSystem进行并行计算，我们可以实现一个如下的结构:
```c#
struct PositionEstimateJob : IJobParallelFor
{
    [ReadOnly] 
    public NativeArray<float3> positions;
    [ReadOnly] 
    public NativeArray<float3> velocities;
    [ReadOnly]
    public NativeArray<float3> normals;
    [ReadOnly]
    public NativeArray<float> masses;
    [WriteOnly]
    public NativeArray<float3> predictPositions;
    public float3 fieldForce;
    public float damper;
    public float dt;

    public void Execute(int index)
    {
      var p = positions[index];
      var v = velocities[index];
      var m = masses[index];
      //....
    }
}
```

JobSystem将会在多线程上并发执行Execute函数，在这里将会针对每个质点执行一次Execute函数，index为当前执行的任务索引(也即质点索引)。由于在这一步每个质点只更新自己的预测位置，不存在Race Condition，因此可以进行无锁并发。

在这里，我们输入的数据有:
* positions - 位置
* velocities - 速度
* normals - 法线
* masses - 质量
* fieldForce - 外部场力,例如风力
* damper - 阻尼系数
*dt - 迭代时间

输出为:
* predictPositions

位置预测公式如下:

![](https://www.zhihu.com/equation?tex=%5Cbegin%7Baligned%7D+%5Cvec%7Bv_1%7D+%3D+%5Cvec%7Bv_0%7D+%2B+%28%5Cvec%7Bg%7D++%2B+%5Cfrac%7B%5Cvec%7BF_%7Be%7D%7D%7D%7Bm%7D%29+%5CDelta+t+%5Cnewline+v_1+%3D+v_1+%2A+%5Cmax%281+-+%5Cfrac%7Bk_d%7D%7Bm%7D+%5CDelta+t%2C0%29+%5Cnewline+p_1+%3D+p_0+%2B+v_1+%2A+%5CDelta+t+%5Cend%7Baligned%7D+)

这三条公式，第一步为预测速度，第二步对速度施加阻尼，其中kd为阻尼系数，第三步利用速度预测位置。

g为重力加速度， Fe为作用于质点的外力，在本例里我们只用到了风力。 风对质点的作用力，需要投影到质点的法线方向，这也是为什么这个Job的输入需要有法线信息。

随风舞动的效果：

// todo

## Collisions

针对球体(Sphere)、方体(Box)、胶囊(Capsule)三种基础类型的碰撞检测。为简单起见，目前采用离散的检测方式，即判断质点是否在相关碰撞体内部。

算法流程为:

```
for position, predictPosition,index in vertices:
  for collider in colliders:
    concat = CheckCollision(position,predictPosition,collider)
    if concat:
      AddCollisionConstraint(index,concat.position,concat.normal)
```
* 遍历所有质点，取其位置和预测位置，与所有的碰撞体做检测
* 如果发现进入碰撞体内部，则计算出碰撞体表面距离当前质点最近的一个点
* 对当前质点施加一个碰撞约束，让其在后面的约束迭代中强行离开碰撞体，回到表面去。


在IntersectUtil这个类里。我们实现了判定一个点是否在碰撞体内部
```c#
public static bool GetClosestSurfacePoint(float3 p, SphereDesc sphere, out ConcatInfo concatInfo)
```

GetClosestSurfacePoint它接收一个点p坐标，和一个碰撞体描述(SphereDesc)，如果发现点在碰撞体内部，就返回true,并且输出一个ConcatInfo结构。

ConcatInfo里保存了碰撞体表面距离p最近一个点的position和normal信息。

我们的目的是让这些碰撞体来约束布料的同时，布料也要反作用于这些碰撞体(假如是RigidBody的话)。

因此我们还需要额外定义一个刚体描述结构:
```c#
public struct RigidbodyDesc{
    public float mass;
    public float bounciness;
    public float3 velocity;
}
```
然后将RigidBody和Collider组合在一起:
```c#
public struct RigidbodyColliderDesc<T> where T:IColliderDesc{
    public int entityId;
    public T collider;
    public RigidbodyDesc rigidbody;
}
```
其中entityId后面会用来查找Unity对象。

然后定义个结构把所有需要与布料进行碰撞检测的数据整合在一起:
```c#
public struct CollidersGroup{
    private NativeList<RigidbodyColliderDesc<SphereDesc>> _spheres;
    private NativeList<RigidbodyColliderDesc<BoxDesc>> _boxes;
    private NativeList<RigidbodyColliderDesc<CapsuleDesc>> _capsules;
}
```

### 球体碰撞

```c#
public struct SphereDesc{
    public float3 center;
    public float radius;
}
```

// todo

### 胶囊碰撞

```c#
public struct CapsuleDesc{
    public float3 c0;
    public float4 axis; //xyz为单位化的方向，w为长度
    public float radius;
}
```

// todo

### 平行六面体

```c#
public struct BoxDesc{
    public float3 min;
    //3个边轴,xyz为normalzied的朝向，w为长度
    public float4 ax; 
    public float4 ay;
    public float4 az;
}
```

使用了xyz-min的角、和3个边向量来定义这个Box，严格来说，它能描述三维空间中的任意平行六面体。其中三个边向量，我们用float4保存，并且做了一些预计算。其xyz分量为单位向量，w分量向量长度。相关的预计算可以加速后面的碰撞检测。

// todo

### Collision Job

我们需要用多线程任务并行执行碰撞检测和反作用力计算。

这个Job的输入输出数据结构
```c#
public struct CollisionJob : IJobParallelFor
{
    [ReadOnly]
    public float dt;
    [ReadOnly]
    public NativeArray<float3> positions;
    [ReadOnly]
    public NativeArray<float3> predictPositions;
    [ReadOnly]
    public NativeArray<float> masses;
    [ReadOnly]
    public CollidersGroup collidersGroup;
    [WriteOnly]
    public NativeArray<CollisionConstraintInfo> collisionConstraints;
    public NativeArray<ConstraintType> constraintTypes;
    public NativeList<RigidBodyForceApply>.ParallelWriter rigidBodyForceApplies;
}
```

它会根据当前质点的predictPosition和position信息，与CollidersGroup中的所有碰撞体进行碰撞检测，如果检测到有碰撞发生，则会做如下事情:

* 修正predictPosition，避免碰撞发生
* 将双方速度投影到接触点法线方向，利用动量守恒、动能公式重新计算双方在法线方向的速度
* 将对Collider的速度修正贡献写入到rigidBodyForceApplies中，供后续实际作用到unity rigidboy对象。
* 将对质点的约束信息写入到collisionConstraints中，供后续约束迭代阶段使用.

在法线方向上，碰撞引起的速度改变符合动量守恒定律

实际上，如果两个物体表面存在摩擦力，那么在垂直于法线的另一个方向上也会产生力的相互作用。但是在当前的这个版本里，我们暂且忽略了。

## Constraints

在布料模拟中，存在如下几种约束:

* 距离约束
* 弯曲约束
* 固定约束
* 碰撞约束

### 距离约束

距离约束就是两个质点之间的距离，必须保持被约束在一定范围之内。

### 弯曲约束

### 固定约束

### 碰撞约束

## Iterations

### 约束迭代

### 更新质点位置和速度

### 布料对外部物体施加力

## Todo

### 连续碰撞检测

### 布料破坏

### 自相交

# Liquid

todo
