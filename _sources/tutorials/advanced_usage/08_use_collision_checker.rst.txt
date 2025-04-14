碰撞检测
=============================

碰撞检测模块基于之前介绍的Environment模块，实现对仿真中机器人和环境之间的碰撞状态检测，除此之外还可以对机器人自身是否碰撞进行检测。
基于碰撞信息可以在真实机器人运动前预测机器人的运动路径是否合法安全，对机器人实现自主运动具有重要意义。

碰撞检测模块的创建与使用
--------------------------

下面代码介绍了如何基于已有Environment创建使用碰撞检测器

.. code-block:: python

    import time
    import numpy as np
    from RVLab import RPS
    from IPython import embed

    env = RPS.Environment()
    rvis = RPS.RobotVis()
    rvis.LoadEnvironment(env)

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
    manip = robot_model.GetActiveManipulator()
    env.AddBody(robot_model)

    container_box1 = RPS.RigidBody()
    container_box1.InitFromContainerBox(length=0.6, width=0.9, height=0.5, alpha_l=0.025, beta_w=0.025, gamma_h=0.025)
    container_box1.SetBaseTransformation(RPS.Pose([0.9,0,0.2,0,0,0,1]))
    env.AddBody(container_box1)

    # 创建碰撞检测器
    checker = RPS.FCLCollisionChecker()
    # 用env初始化碰撞检测器
    checker.InitFromEnv(env)

    while True:
        res, report = checker.CheckCollision(env.GetCollisionMatrix())
        if res:
            print("collision")
        else:
            print("free")
        
        time.sleep(0.1)

    embed()

.. image:: ../../_static/imgs/08_use_collision_checker_00.gif
   :alt: 碰撞检测
   :align: center

