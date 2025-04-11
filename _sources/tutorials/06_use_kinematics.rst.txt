运动学求解器
=============================

机器人模型 :doc:`03_use_robotmodel` 中包含了机器人所有的关节和连杆参数，除了用于仿真显示，还会被用于计算机器人的正向运动学、逆向运动学、雅可比矩阵等。
为方便地实现上述功能，RPCP中提供了Kinematics运动学模块，对于绝大部分工业串联六轴球腕型机械臂或者UR构型的机械臂，RPCP中提供了 **解析解** ，相较于数值迭代求解，速度更快稳定性更高。
对于部分协作七轴机械臂，RPCP还可以根据不同的需求返回不止一组数值解供开发者灵活选择。在 **高级功能章节** 还会展开介绍带外部行走轴的运动学求解器使用方法。 

正向运动学
-----------------------
以下代码以常见的工业串联六轴球腕型机械臂Motoman_GP12为例，介绍如何使用RPCP的正向运动学

.. code-block:: python

    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
    manip = robot_model.GetActiveManipulator()
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 将机器人移动到某一预定位置
    q0 = RPS.Rx([0, -0.0440593, -0.448863, 0, -1.166, 0])
    manip.SetJointPositions(q0)

    # 创建运动学求解器
    kin_solver = RPS.CreateKinSolver(manip)

    # 运动学正解：计算q0位置时机器人的末端位置与姿态
    res, pose = kin_solver.GetPositionFK(q0)
    # 计算出的pose为([0.794998, 0, 0.895184, 1, 0, -3.6866e-06, 0])

    # 检查计算是否成功
    assert res == RPS.RVSReturn_Success
    embed()

    # 将计算得到的Pose在仿真器中绘制，观察是否与当前机器人末端位置姿态重合
    h = rvis.PlotFrame(pose,axis_len=0.2,axis_size=5)
    embed()
    rvis.Delete(h)

    embed()

逆向运动学
-----------------------
以下代码以常见的工业串联六轴球腕型机械臂Motoman_GP12为例，介绍如何使用RPCP的逆向运动学

.. code-block:: python

    import time 
    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
    # robot_model.InitFromFile('./source/_static/model/RobotModel/UR5.robot')
    manip = robot_model.GetActiveManipulator()
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 创建运动学求解器
    kin_solver = RPS.CreateKinSolver(manip)

    # GP12末端目标位置
    target_pose = RPS.Pose([-0.016855, 0.428374, 1.48864, -0.647242, -0.174703, -0.283169, 0.685836])
    # UR5末端目标位置
    # target_pose = RPS.Pose([-0.167369, -0.19145, 0.569015, 0.707107, 0, 0, 0.707107])

    # 解析解：求解所有满足要求的关节位置
    res, ik_report = kin_solver.GetPositionIK(target_pose)

    # 检查计算是否成功
    assert res == RPS.RVSReturn_Success
    embed()

    # 打印逆解的个数,该案例中GP12应该总共是28组逆解，UR5应该总共是6组逆解
    print("关节极限范围内一共存在{}组逆解".format(ik_report.ik_number))

    # 验证逆解的正确性
    # 方法1：将每个逆解重新计算正解，验证计算的正解是否和target_pose一致
    # 方法2：直接利用仿真器验证
    for j in range(5):
        for i in range(ik_report.ik_number):
            manip.SetJointPositions(ik_report[i])
            time.sleep(0.1)

    embed()

.. image:: ../_static/imgs/06_use_kinematics_00.gif
   :alt: 解析解
   :align: center

.. image:: ../_static/imgs/06_use_kinematics_01.gif
   :alt: 解析解
   :align: center

逆向运动学(最近逆解)
-----------------------
对于绝大部分使用场景可能不需要返回所有的运动学逆解，而是希望返回距离机器人当前位置最近的一组逆解。
一种解决方式是对返回的多组关节位置和机器人当前位置做距离计算，手动选择最近的逆解。另一种方法是直接
采用RPCP中提供的求最近逆解的功能

.. code-block:: python

    import time 
    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
    manip = robot_model.GetActiveManipulator()
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 创建运动学求解器
    kin_solver = RPS.CreateKinSolver(manip)

    # GP12末端目标位置
    target_pose = RPS.Pose([-0.016855, 0.428374, 1.48864, -0.647242, -0.174703, -0.283169, 0.685836])

    rvis.PlotFrame(target_pose,axis_len=0.2,axis_size=5)

    # 机器人当前位置,希望找到的逆解是距离当前位置最近的一组逆解
    curr_q = manip.GetJointPositions()

    # 最近逆解，返回值分别表示是否成功，ik信息，与输入位置的距离
    res, ik_report, dist = kin_solver.GetNearestIK(target_pose,curr_q)

    # 检查计算是否成功
    assert res == RPS.RVSReturn_Success
    embed()

    # 最近逆解
    target_q = ik_report[0]

    # 仿真器插值显示从当前位置到目标位置的运动过程
    for i in range(100):
        q = curr_q + (target_q - curr_q) * i / 100.0
        manip.SetJointPositions(q)
        time.sleep(0.05)
    
    embed()

.. image:: ../_static/imgs/06_use_kinematics_02.gif
   :alt: 最近逆解
   :align: center


逆向运动学(七轴数值解)
-----------------------
对于七轴机械臂，除了可以使用最近逆解获取一个相对较优的逆解，也可以通过RPCP获取多组逆解，供开发者选择

.. code-block:: python

    import time 
    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/Franka_fer_arm.robot')
    manip = robot_model.GetActiveManipulator()
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 创建运动学求解器
    kin_solver = RPS.CreateKinSolver(manip)

    # GP12末端目标位置
    target_pose = RPS.Pose([0.441891, 0.18907, 0.459471, 0.733074, -0.460541, -0.154862, -0.475943])

    rvis.PlotFrame(target_pose,axis_len=0.2,axis_size=5)

    # 机器人当前位置,希望找到的逆解是距离当前位置最近的一组逆解
    curr_q = manip.GetJointPositions()

    # 最近逆解，返回值分别表示是否成功，ik信息，与输入位置的距离
    res, ik_report, dist = kin_solver.GetNearestIK(target_pose,curr_q)

    # 最近逆解
    target_q = ik_report[0]

    embed()
    # 仿真器插值显示从当前位置到目标位置的运动过程
    for i in range(100):
        q = curr_q + (target_q - curr_q) * i / 100.0
        manip.SetJointPositions(q)
        time.sleep(0.05)

    # 查找返回多组逆解
    res, ik_report = kin_solver.SearchAllPositionIK(target_pose)

    # 打印找到的逆解的个数
    print("找到{}组逆解".format(ik_report.ik_number))

    for j in range(5):
        for i in range(ik_report.ik_number):
            manip.SetJointPositions(ik_report[i])
            time.sleep(0.1)
    
    embed()

.. image:: ../_static/imgs/06_use_kinematics_03.gif
   :alt: 七轴机械臂逆解
   :align: center


计算雅可比矩阵
-----------------------
以下代码先计算机器人末端连杆的雅可比矩阵，再通过雅可比矩阵，控制机器人末端做直线运动

.. code-block:: python

    import time 
    import numpy as np
    from RVLab import RPS
    from IPython import embed

    robot_model = RPS.RobotModel()
    robot_model.InitFromFile('./source/_static/model/RobotModel/UR5.robot')
    manip = robot_model.GetActiveManipulator()
    manip.SetJointPositions(RPS.Rx([0, -1.73478, 1.32797, 0.406815, 0, 0]))
    rvis = RPS.RobotVis()
    rvis.AddBody(robot_model)

    # 创建运动学求解器
    kin_solver = RPS.CreateKinSolver(manip)

    # 计算末端雅可比矩阵
    res, jaco = kin_solver.GetGeomJacobianWrtSpace(manip.GetJointPositions())
    print("jaco: \n", jaco)

    embed()
    # 通过雅可比的逆控制机器人运动 
    for i in range(100):
        res, jaco = kin_solver.GetGeomJacobianWrtSpace(manip.GetJointPositions())
        delta_q = np.linalg.pinv(jaco) @ np.array([0,0,-0.01,0,0,0])
        manip.SetJointPositions(manip.GetJointPositions()+delta_q)
        time.sleep(0.02)

    embed()

.. image:: ../_static/imgs/06_use_kinematics_04.gif
   :alt: 计算雅可比矩阵
   :align: center
