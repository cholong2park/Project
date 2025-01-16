import rclpy
import DR_init
from DR_common2 import posx, posj
import numpy as np

ROBOT_ID= "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC =50, 50
VELOCITY_J, ACC_J =10, 10
DR_init.__dsr__id =ROBOT_ID
DR_init.__dsr__model=ROBOT_MODEL
ON, OFF = 1, 0

grap_x=[363.417, -177.423, 193.191, 154.666, -178.59, 154.287]
# 1단:
point=[[398.89, 152.132, 80, 146.913, -179.193, 146.62]]
# 4단:
fourth_ready_j=[-54.602, 49.55, 77.547, 49.763, 129.809, -51.77]
fourth_grap_j = [-54.185, 60.405, 75.728, 45.504, 123.674, -59.611]
fourth_middle_j=[-0.165, 4.414, 60.05, 0.028, 115.542, -0.017]
fourth_x= [445.85, 40.311, 309.734, 95.693, -90.003, 89.998]
root_3=3**(1/2)
delta = [[38*root_3, -38, 0, 0, 0, 0],
[76*root_3, -76, 0, 0, 0, 0],
[0, -76, 0, 0, 0, 0],
[38*root_3, -114, 0, 0, 0, 0],
[0, -152, 0, 0, 0, 0],
[38*root_3/3, -38, 96, 0, 0, 0],
[38*root_3*4/3, -76, 96, 0, 0, 0],
[38*root_3/3, -114, 96, 0, 0, 0],
[76*root_3/3, -76, 192, 0, 0, 0]]

JReady=[0,0,90,0,90,0]
def main(args=None):
    rclpy.init(args=args)
    node=rclpy.create_node("rokey_pick_and_place",namespace=ROBOT_ID)
    DR_init.__dsr__node=node
    try:
        from DSR_ROBOT2 import(
            set_digital_output,
            set_tool,
            set_tcp,
            movesx,
            movel,
            movej,
            wait,
            trans,
            DR_BASE,
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)
    def pick_up_put_down(n):
        real_grap=grap_x[:]
        real_grap[2]-=11.2*n
        real_grap_up=real_grap[:]
        real_grap_up[2]+=120
        put_up=point[n][:]
        put_up[2]+=100

        movel(real_grap,vel=VELOCITY,acc=ACC)
        grip()
        movesx([posx(real_grap_up),posx(put_up),posx(point[n])],vel=VELOCITY,acc=ACC)
        release()

    set_tcp("2FG_TCP")
    set_tool("Tool Weight_2FG")
    while rclpy.ok():
        for i in range(9):
            add=trans(point[0], delta[i], DR_BASE, DR_BASE)
            add = add.tolist()
            point.append(add)
        print(point)
        release()
        movej(JReady, vel=VELOCITY_J, acc=ACC_J)
        for i in range(10):
            pick_up_put_down(i)
        movej(fourth_ready_j, vel=VELOCITY_J, acc=ACC_J)
        movej(fourth_grap_j, vel=VELOCITY_J, acc=ACC_J)
        grip()
        movej(fourth_middle_j, vel=VELOCITY_J, acc=ACC_J)
        fourth_up=fourth_x[:]
        fourth_up[2]+=50
        movel(posx(fourth_x),vel=VELOCITY,acc=ACC)
        release()
        rclpy.shutdown()
if __name__=="__main__":
    main()