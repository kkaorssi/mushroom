from indy_utils import indydcp_client as client

## Robot info
robot_ip = "192.168.0.119" # indy 로봇의 IP
robot_name = "NRMK-Indy7"
indy = client.IndyDCPClient(robot_ip, robot_name)

indy.connect()

# 로봇의 현재 상태 확인
status = indy.get_robot_status()
print(status)

j_pos = indy.get_joint_pos()
print(j_pos)

t_pos = indy.get_task_pos()
print(t_pos)

# T_SNAPSHOT = [-0.280, -0.032, 0.5000, 0, 180, 0]
# indy.task_move_to(T_SNAPSHOT)
# indy.reset_robot()
indy.go_home()

indy.disconnect()