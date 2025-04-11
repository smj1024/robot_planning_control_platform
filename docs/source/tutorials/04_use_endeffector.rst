EndEffector
===================

RPCP中采用 **RPS.EndEffector** 管理机器人末端执行器

创建并使用EndEffector
-----------------------
以下代码展示了在Motoman_GP12机器人末端安装一个末端执行器吸盘

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

    # 创建新的末端执行器EndEffector对象
    eef = RPS.EndEffector()
    # 末端执行器从二进制文件进行初始化
    eef.InitFromFile('./source/_static/model/EndEffector/FourFingerVacuumPads.eef')
    embed()
    
    # 将末端执行器安装到manip末端
    manip.SetActiveEndEffector(eef)
    embed()

    # 设置末端执行器的安装位置姿态
    eef.SetAttachingPose(RPS.Pose(0,0,0,0,0,0))
    embed()

    # 打印安装末端执行器后的manip的TCP位置姿态
    print(manip.GetTCP())

    # 设置manip的TCP位置姿态
    manip.SetTCP(RPS.Pose([0.185, 0, 0.28, 0, 1, 0, 0]))
    embed()

    # 显示碰撞模型
    rvis.ChooseShowMode(mode=2)
    embed()

    # 显示可视化模型
    rvis.ChooseShowMode(mode=1)
    embed()

    # 在仿真器中交互式拖动机器人
    rvis.StartDragging(manip)
    embed()

    # 停止交互式拖动机器人
    rvis.StopDragging()
    embed()

.. image:: ../_static/imgs/04_use_endeffector_00.gif
   :alt: 使用EndEffector
   :align: center