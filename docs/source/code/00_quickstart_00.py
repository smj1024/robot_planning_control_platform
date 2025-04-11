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
