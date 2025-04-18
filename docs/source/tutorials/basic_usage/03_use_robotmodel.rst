RobotModel
===================

RPCP中采用 **RPS.RobotModel** 管理机器人模型和参数

创建并显示RobotModel
-----------------------

.. code-block:: python

    import numpy as np
    from RVLab import RPS
    from IPython import embed

    # 创建空的RobotModel
    robot_model = RPS.RobotModel()
    # 从二进制文件初始化机器人RobotModel
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')

    rvis = RPS.RobotVis()
    # 将机器人RobotModel添加到运动学仿真器中
    rvis.AddBody(robot_model)
    embed()

    # 设置Robotodel的基座位置姿态
    robot_model.SetBaseTransformation(RPS.Pose(0.5,0,0,0,0,np.pi/2))
    embed()

    # 查询打印RobotModel的基座位置姿态
    print(robot_model.GetBaseTransformation())
    embed()

    # 查询打印RobotModel的名字
    print(robot_model.GetName())
    embed()

    # 显示碰撞模型
    rvis.ChooseShowMode(mode=2)
    embed()

    # 显示可视化模型
    rvis.ChooseShowMode(mode=1)
    embed()

.. image:: ../../_static/imgs/03_use_robotmodel_00.gif
   :alt: RobotModel
   :align: center

使用Manipulator
-------------------
Manipulator是RobotModel中某些关节和连杆组成的一条链，对于绝大部分普通单臂机器人，Manipulator包含RobotModel从基座到末端连杆，这种情况下RobotModel只有一个Manipulator(当然也可以定义多个Manipulator，比如再定义一个从基座到第4连杆的Manipulator)。对于双臂机器人的RobotModel，至少存在两个Manipulator。

以下代码展示了Manipulator的使用方法

.. code-block:: python

    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 从RobotModel获取Manipulator
    manip = robot_model.GetActiveManipulator()

    # 打印manip的自由度
    print("manip dof: ", manip.GetDoF())

    # 打印manip的关节权重
    print("manip joint weights: ", manip.GetJointWeights())

    # 打印manip的关节约束(位置、速度、加速度约束)
    print("manip joint limits: ", manip.GetJointLimits())

    # 打印manip的当前关节位置
    print("manip current joint positions: ", manip.GetJointPositions())

    # 打印manip的关节原位
    print("manip home joint positions: ", manip.GetHomePositions())

    # 设置manip的关节位置
    manip.SetJointPositions(RPS.Rx([1.5708,0,0,0,-1.5708,0]))

    # 设置manip的关节原位
    manip.SetHomePositions(RPS.Rx([0,0,0,0,-1.5708,0]))

    # 打印manip的tcp位置姿态信息
    print("manip tcp: ", manip.GetTCP())

    # 设置manip的tcp位置姿态
    manip.SetTCP(RPS.Pose(0,0,0.1,0,0,0))

