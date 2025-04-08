快速开始
=========
这一教程主要是以几个简单的场景为例，快速介绍RPCP机器人开发平台的功能和用法，让读者对RPCP有宏观大致的了解，更多功能以及每个功能的详细使用方法参考剩余教程

工件的抓取与搬运
-------------------------
运行以下python程序，仿真中GP12机器人将依次将9个工件搬运到目标位置

.. code-block:: python

   from RVLab import RPS
   from IPython import embed

   # 打开仿真器
   rvis = RPS.RobotVis()

   # 终端输入exit()或者ctrl+d继续
   embed()

   # 创建桌子1模型
   table1 = RPS.RigidBody()
   table1.InitFromBox(0.5, 0.5, 0.6)
   table1.SetBaseTransformation(RPS.Pose(1, 0, 0.3, 0, 0, 0))
   rvis.AddBody(table1)

   # 创建桌子2模型
   table2 = RPS.RigidBody()
   table2.InitFromBox(0.5, 0.5, 0.6)
   table2.SetBaseTransformation(RPS.Pose(0, 1, 0.3, 0, 0, 0))
   rvis.AddBody(table2)

   # 创建9个被抓取工件
   workpieces = []
   target_pose = []
   for i in range(3):
      for j in range(3):
         box = RPS.RigidBody()
         box.InitFromBox(0.09, 0.09, 0.05)
         rvis.AddBody(box)
         rvis.SetColor(box, [1, 0, 0, 1])
         workpieces.append(box)
         workpieces[-1].SetBaseTransformation(RPS.Pose(0.8 + i * 0.2, -0.2 + j * 0.2, 0.625, 0, 0, 3.1415926))
         # 创建9个放置目标位置
         target_pose.append(RPS.Pose(-0.2 + i * 0.2, 0.8 + j * 0.2, 0.625, 0, 0, -1.5708))

   # 创建GP12机器人
   robot_model = RPS.RobotModel()
   robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
   rvis.AddBody(robot_model)

   # 创建末端执行器(吸盘)
   eef = RPS.EndEffector()
   eef.InitFromFile('./source/_static/model/EndEffector/FourFingerVacuumPads.eef')
   eef.SetAttachingPose(RPS.Pose(0, 0, 0, 0, 0, 0))

   # 机器人安装末端执行器
   manip = robot_model.GetActiveManipulator()
   manip.SetActiveEndEffector(eef)

   # 创建仿真虚拟控制器
   controller = RPS.SimController.Create(manip)
   # 连接机器人
   controller.Connect('127.0.0.1')
   # 使能机器人
   controller.EnableRobot()
   # 创建基础运动规划器
   base_motion = RPS.BasicMotionUtilities(controller)
   base_motion.SetJointThreshold(10)

   for i in range(3):
      for j in range(3):
         base_motion.MoveJoints(RPS.SE3Tangent(0, 0, 0.125, 0, 0, 0) + workpieces[i * 3 + j].GetBaseTransformation())
         base_motion.MoveBZ(-0.1)
         eef.Grab(workpieces[i * 3 + j])
         base_motion.MoveBZ(0.1)
         base_motion.MoveJoints(RPS.SE3Tangent(0, 0, 0.125, 0, 0, 0) + target_pose[i * 3 + j])
         base_motion.MoveBZ(-0.1)
         eef.Release(workpieces[i * 3 + j])
         base_motion.MoveBZ(0.1)

   # 回原位
   base_motion.MoveJoints(RPS.Rx([0, 0, 0, 0, -1.5708, 0]))

   embed()

.. image:: ../_static/imgs/00_quickstart_00.gif
   :alt: GP12抓取搬运案例 
   :align: center


带避障功能的工件抓取与放置
------------------------------------
运行以下python程序，仿真中GP12机器人将依次将9个工件搬运到目标位置，同时对于搬运路径上的障碍物，自动生成无碰撞的路径

.. code-block:: python

   from RVLab import RPS
   from IPython import embed

   # 打开仿真器
   rvis = RPS.RobotVis()

   # 终端输入exit()或者ctrl+d继续
   embed()

   # 创建桌子1模型
   table1 = RPS.RigidBody()
   table1.InitFromBox(0.5, 0.5, 0.6)
   table1.SetName("table1")
   table1.SetBaseTransformation(RPS.Pose(1, 0, 0.3, 0, 0, 0))
   rvis.AddBody(table1)

   # 创建桌子2模型
   table2 = RPS.RigidBody()
   table2.InitFromBox(0.5, 0.5, 0.6)
   table2.SetName("table2")
   table2.SetBaseTransformation(RPS.Pose(0, 1, 0.3, 0, 0, 0))
   rvis.AddBody(table2)

   # 创建搬运路径上的障碍物
   wall = RPS.RigidBody()
   wall.InitFromBox(0.1, 0.7, 0.9)
   wall.SetName("wall")
   wall.SetBaseTransformation(RPS.Pose(0.65, 0.65, 0.45, 0, 0, -1.5708 / 2.0))
   rvis.AddBody(wall)

   # 创建9个被抓取工件
   workpieces = []
   target_pose = []
   for i in range(3):
      for j in range(3):
         box = RPS.RigidBody()
         box.InitFromBox(0.09, 0.09, 0.05)
         box.SetName("box" + str(i * 3 + j))
         rvis.AddBody(box)
         rvis.SetColor(box, [1, 0, 0, 1])
         workpieces.append(box)
         workpieces[-1].SetBaseTransformation(RPS.Pose(0.8 + i * 0.2, -0.2 + j * 0.2, 0.625, 0, 0, 3.1415926))
         # 创建9个放置目标位置
         target_pose.append(RPS.Pose(-0.2 + i * 0.2, 0.8 + j * 0.2, 0.625, 0, 0, -1.5708))

   # 创建GP12机器人
   robot_model = RPS.RobotModel()
   robot_model.InitFromFile('./source/_static/model/RobotModel/GP12.robot')
   rvis.AddBody(robot_model)

   # 创建末端执行器(吸盘)
   eef = RPS.EndEffector()
   eef.InitFromFile('./source/_static/model/EndEffector/FourFingerVacuumPads.eef')
   eef.SetAttachingPose(RPS.Pose(0, 0, 0, 0, 0, 0))

   # 机器人安装末端执行器
   manip = robot_model.GetActiveManipulator()
   max_vel, max_acc, max_jerk = RPS.ConvertJointLimitsToCVec(manip.GetJointLimits())
   manip.SetActiveEndEffector(eef)

   # 创建仿真虚拟控制器
   controller = RPS.SimController.Create(manip)
   # 连接机器人
   controller.Connect('127.0.0.1')
   # 使能机器人
   controller.EnableRobot()
   # 创建基础运动规划器
   base_motion = RPS.BasicMotionUtilities(controller)
   base_motion.SetJointThreshold(10)

   # 创建运动学求解器
   kin_solver = controller.GetKinSolver()

   # 创建点到点规划器
   p2p_planner = RPS.OmplParallelPlanPlanner(planner_types=4 * [RPS.OmplType_RRTConnect],
                                             max_solutions=10,
                                             optimize=False,
                                             max_planning_time=5.0,
                                             simplify=True,
                                             simplify_level=RPS.Simplify_Level_Medium,
                                             smooth=True)
   planner_config = RPS.OmplParallelPlanPlanner.GetDefaultJson()
   p2p_planner.SetPlannerComponents(planner_config)
   p2p_planner.Config(rvis.m_env, manip)

   for i in range(3):
      for j in range(3):
         # 将机器人当前位置设置为规划的起点
         start = controller.GetJointPositions()[1]
         # 计算规划的终点位姿
         goal_pose = RPS.SE3Tangent(0, 0, 0.125, 0, 0, 0) + workpieces[i * 3 + j].GetBaseTransformation()
         # 用运动学求解器计算对应的逆解
         res = kin_solver.GetNearestIK(goal_pose, start)
         if res[0] != RPS.RVSReturn_Success:
               print("IK failed")
               exit()
         # 更新规划器的env
         p2p_planner.Config(rvis.m_env, manip)
         # 创建规划请求
         request = RPS.OmplPlanner.CreateRequest(start, res[1][0])
         # 规划计算
         res, response = p2p_planner.Solve(request, False)
         if not res:
               print("Plan failed")
               exit()

         # 轨迹规划生成连续光滑路径
         path = RPS.CreatePath(response.joint_positions_vec, 0.1, RPS.PathType_Bezier2ndBlend)
         traj = RPS.CreateTrajectory(path, max_vel, max_acc, max_jerk, traj_type=RPS.TrajType_Trapezoidal)
         hs = RPS.DrawTraj(traj=traj,
                           show_in_vis=True,
                           show_in_plt=False,
                           view=rvis.m_view,
                           kin_solver=kin_solver,
                           axis_length=0.02)
         # 修改控制器速度为0.2
         controller.SetSpeedRatio(0.2)
         # 执行轨迹
         controller.ExecuteTrajectory(traj)
         # 修改控制器速度为1
         controller.SetSpeedRatio(1)
         rvis.Delete(hs)
         # 机器人末端下降0.1m
         base_motion.MoveBZ(-0.1)
         # 仿真中抓取工件
         eef.Grab(workpieces[i * 3 + j])
         # 机器人末端抬升0.1m
         base_motion.MoveBZ(0.1)

         start = controller.GetJointPositions()[1]
         goal_pose = RPS.SE3Tangent(0, 0, 0.125, 0, 0, 0) + target_pose[i * 3 + j]
         res = kin_solver.GetNearestIK(goal_pose, start)
         if res[0] != RPS.RVSReturn_Success:
               print("IK failed")
               exit()
         p2p_planner.Config(rvis.m_env, manip)
         request = RPS.OmplPlanner.CreateRequest(start, res[1][0])
         res, response = p2p_planner.Solve(request, False)
         if not res:
               print("Plan failed")
               exit()

         path = RPS.CreatePath(response.joint_positions_vec, 0.1, RPS.PathType_Bezier2ndBlend)
         traj = RPS.CreateTrajectory(path, max_vel, max_acc, max_jerk, traj_type=RPS.TrajType_Trapezoidal)
         hs = RPS.DrawTraj(traj=traj,
                           show_in_vis=True,
                           show_in_plt=False,
                           view=rvis.m_view,
                           kin_solver=kin_solver,
                           axis_length=0.02)
         controller.SetSpeedRatio(0.2)
         controller.ExecuteTrajectory(traj)
         controller.SetSpeedRatio(1)
         rvis.Delete(hs)
         base_motion.MoveBZ(-0.1)
         eef.Release(workpieces[i * 3 + j])
         base_motion.MoveBZ(0.1)

   # 回原位
   base_motion.MoveJoints(RPS.Rx([0, 0, 0, 0, -1.5708, 0]))

   embed()

.. image:: ../_static/imgs/00_quickstart_01.gif
   :alt: GP12避障抓取搬运案例 
   :align: center