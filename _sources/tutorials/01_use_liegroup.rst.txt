基础数据类型
==================

关节位置
-----------
RPCP中采用 **RPS.Rx** 数据类型来表示机器人的关节位置

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    # 用list定义一个六自由度机器人关节位置q0
    q0 = RPS.Rx([0,0,0,0,0,0])

    # 用numpy数据类型定义一个六自由度机器人关节位置q1
    q1 = RPS.Rx(np.array([0.1,0.2,0.3,0.4,0.5,0.6]))

    # 将Rx转化成numpy的数据类型
    a = q1.Coeffs()

    # 输出q0自由度
    print(q0.DoF())

    # 随机生成一个7轴机器人关节位置数据
    q_random = RPS.Rx.RandomStatic(7)
    print(q_random)

    # 关节插值计算
    q = q0 + 0.5*(q1-q0)

笛卡尔位置姿态
---------------
RPCP中采用 **RPS.Pose** 数据类型表示机器人的末端位置与姿态

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    # 定义一个位置在世界坐标系原点，姿态与世界坐标系重合的pose
    p0 = RPS.Pose()
    # 输出：Pose([0, 0, 0, 0, 0, 0, 1])
    # 前三位为位置参数：x，y，z
    # 后四位为四元数：ox，oy，oz，ow

    # 定义一个随即pose
    p0 = RPS.Pose.RandomStatic()

    # 利用位置+四元数定义一个pose
    p1 = RPS.Pose(0.1,0.2,0.3,0,0,0,1)
    # 前三位为位置参数：x，y，z
    # 后四位为四元数：ox，oy，oz，ow

    # 利用位置+四元数定义一个pose
    p1 = RPS.Pose(np.array([0.1,0.2,0.3,0,0,0,1]))
    # 前三位为位置参数：x，y，z
    # 后四位为四元数：ox，oy，oz，ow

    # 利用位置+四元数定义一个pose
    p1 = RPS.Pose(np.array([0.1,0.2,0.3]),np.array([0,0,0,1]))
    # 前三位为位置参数：x，y，z
    # 后四位为四元数：ox，oy，oz，ow

    # 从一个已有pose构造一个新pose
    p2 = RPS.Pose(p1)

    # 从4x4齐次变换矩阵构造一个pose
    p3 = RPS.Pose(np.array([[1,0,0,0.1],
                            [0,1,0,0.2],
                            [0,0,1,0.3],
                            [0,0,0,1]]))

    # 利用位置+RPY角定义一个pose
    p4 = RPS.Pose(0.1,0.2,0.3,0,0,0)
    # 前三位为位置参数：x，y，z
    # 后三位为RPY：roll,pitch,yaw
    # 旋转顺序为rot(yaw)*rot(pitch)*rot(roll)

Pose数据可以通过以下方式访问

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    # 利用位置+RPY角定义一个pose
    p = RPS.Pose(0.1,0.2,0.3,0,0,1.5708)

    # 通过下标访问
    x = p[0]
    y = p[1]
    z = p[2]
    ox = p[3]
    oy = p[4]
    oz = p[5]
    ow = p[6]

    # 将pose转化成numpy数据
    p = p.Coeffs()

    # 获取位置向量
    pos = p.GetR3()

    # 获取位置向量(功能与GetR3相同，但是返回numpy的数据类型)
    pos = p.Translation()

    # 获取四元数
    quat = p.GetSO3()

    # 获取对应的3x3旋转矩阵(numpy的数据类型)
    r_rot = p.Rotation()

    # 获取对应的4x4齐次矩阵(numpy的数据类型)
    T = p.Transform()

    # 获取旋转矩阵对应的RPY角(numpy的数据类型)
    rpy = p.RPY()



空间位姿变换计算
-----------------

已知两个坐标系1和2分别在世界坐标系0下的位姿为p_0_1和p_0_2，求坐标系2在坐标系1中的位置姿态p_1_2

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    # xyz+rpy构造一个pose
    p_0_1 = RPS.Pose(0.2,0,0,0,0,0)

    # xyz+rpy构造一个pose
    p_0_2 = RPS.Pose(0.2,0.3,0,0,0,0)

    # Inverse()函数可以计算4x4齐次矩阵的逆
    p_1_2 = p_0_1.Inverse() * p_0_2
    # 检查p_1_2是否是Pose([0, 0.3, 0, 0, 0, 0, 1])


已知坐标系1在世界坐标系0下的位姿为p_0_1，以下代码展示如何通过 **RPS.Pose** 对坐标系1进行位姿变换

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    p_0_1 = RPS.Pose(0.1,0,0,0,0,np.pi/2)

    # 将p_0_1沿着坐标系1自己的Y轴正方向运动0.2m(右乘)
    p = p_0_1 * RPS.Pose(0,0.2,0,0,0,0)
    # 将p_0_1绕着坐标系1自己的Y轴负方向旋转0.2rad(右乘)
    p = p_0_1 * RPS.Pose(0,0,0,0,-0.2,0)
    # 将p_0_1先沿着坐标系1自己的Y轴正方向运动0.2m，得到新坐标系2后，再绕着坐标系2自己的Y轴负方向旋转0.2rad
    p = p_0_1 * RPS.Pose(0,0.2,0,0,0,0) * RPS.Pose(0,0,0,0,-0.2,0)

    # 将p_0_1沿着世界坐标系0的Y轴正方向运动0.2m(左乘)
    p = RPS.Pose(0,0.2,0,0,0,0) * p_0_1
    # 将p_0_1绕着世界坐标系0的Y轴负方向旋转0.2rad(左乘)
    p = RPS.Pose(0,0,0,0,-0.2,0) * p_0_1
    # 将p_0_1先沿着世界坐标系的Y轴正方向运动0.2m，得到新坐标系2后，再绕着世界坐标系的Y轴负方向旋转0.2rad
    p = RPS.Pose(0,0,0,0,-0.2,0) * (RPS.Pose(0,0.2,0,0,0,0) * p_0_1)

已知坐标系1在世界坐标系0下的位姿为p_0_1，以下代码展示如何通过 **RPS.SE3Tangent** 对坐标系1进行位姿变换

.. code-block:: python

    import numpy as np
    from RVLab import RPS

    p_0_1 = RPS.Pose(0.1,0,0,0,0,np.pi/2)

    # 将p_0_1沿着坐标系1自己的Y轴正方向运动0.2m(右乘)
    p = p_0_1 + RPS.SE3Tangent(0,0.2,0,0,0,0)
    # 将p_0_1绕着坐标系1自己的Y轴负方向旋转0.2rad(右乘)
    p = p_0_1 + RPS.SE3Tangent(0,0,0,0,-0.2,0)
    # 将p_0_1先沿着坐标系1自己的Y轴正方向运动0.2m，得到新坐标系2后，再绕着坐标系2自己的Y轴负方向旋转0.2rad
    p = p_0_1 + RPS.SE3Tangent(0,0.2,0,0,0,0) + RPS.SE3Tangent(0,0,0,0,-0.2,0)

    # 将p_0_1沿着世界坐标系0的Y轴正方向运动0.2m(左乘)
    p = RPS.SE3Tangent(0,0.2,0,0,0,0) + p_0_1
    # 将p_0_1绕着世界坐标系0的Y轴负方向旋转0.2rad(左乘)
    p = RPS.SE3Tangent(0,0,0,0,-0.2,0) + p_0_1
    # 将p_0_1先沿着世界坐标系的Y轴正方向运动0.2m，得到新坐标系2后，再绕着世界坐标系的Y轴负方向旋转0.2rad
    p = RPS.SE3Tangent(0,0,0,0,-0.2,0) + (RPS.SE3Tangent(0,0.2,0,0,0,0) + p_0_1)