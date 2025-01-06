import rclpy
import DR_init
from DR_common2 import posx, posj
ROBOT_ID= "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC =60, 60
DR_init.__dsr__id =ROBOT_ID
DR_init.__dsr__model=ROBOT_MODEL
ON, OFF = 1, 0
tray1=[
[346.473, 149.186, 23.457, 50.584, -178.66, 50.364],
[398.3905, 148.2835, 23.6555, 47.8575, -178.4625, 47.7795],
[450.308, 147.381, 23.854, 45.131, -178.265, 45.195],
[346.895, 98.171, 23.4805, 51.7335, -178.062, 51.5365],
[398.452, 97.4625, 23.621, 49.88725, -177.995, 49.83425],
[450.009, 96.754, 23.7615, 48.041, -177.928, 48.132],
[347.317, 47.156, 23.504, 52.883, -177.464, 52.709],
[398.5135, 46.6415, 23.5865, 51.917, -177.5275, 51.889],
[449.71, 46.127, 23.669, 50.951, -177.591, 51.069]]
tray2=[
[346.175, -52.307, 22.925, 58.634, -177.364, 58.626],
[397.532, -52.514, 22.9275, 59.825, -177.339, 59.7315],
[448.889, -52.721, 22.93, 61.016, -177.314, 60.837],
[345.8875, -103.7, 22.9265, 63.6785, -177.3675, 63.485],
[397.30425, -103.8385, 23.004, 63.72525, -177.338, 63.54875],
[448.721, -103.977, 23.0815, 63.772, -177.3085, 63.6125],
[345.6, -155.093, 22.928, 68.723, -177.371, 68.344],
[397.0765, -155.163, 23.0805, 67.6255, -177.337, 67.366],
[448.553, -155.233, 23.233, 66.528, -177.303, 66.388]]
tray1_z=[0,0,0,0,0,0,0,0,0]
tray2_z=[0,0,0,0,0,0,0,0,0]
six_x=[346.2, -133.349, 31.359, 89.866, 89.254, 91.046]
six_j=[-55.089, 63.689, 78.214, 42.047, 121.009, -64.534]
six_up_x=[346.669, -131.316, 118.322, 89.843, 90.891, 91.485]
six_up_j=[-54.949, 53.291, 80.794, 44.829, 125.164, -58.072]
# six_up_x=[346.029, -133.134, 64.037, 90.053, 89.499, 91.011]
# six_up_j=[-55.001, 59.962, 79.326, 42.917, 122.759, -62.608]
JReady=[0,0,90,0,90,0]
def main(args=None):
    rclpy.init(args=args)
    node=rclpy.create_node("rokey_pick_and_place",namespace=ROBOT_ID)
    DR_init.__dsr__node=node
    try:
        from DSR_ROBOT2 import(
            set_digital_output,
            get_digital_input,
            set_desired_force,
            task_compliance_ctrl,
            release_compliance_ctrl,
            check_force_condition,
            set_tool,
            set_tcp,
            movel,
            movej,
            wait,
            get_current_posx,
            DR_BASE,
            DR_AXIS_Z,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL
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
    def pick(up,down):
        movel(posx(up),vel=VELOCITY,acc=ACC)
        movel(posx(down),vel=VELOCITY,acc=ACC)
        grip()
        movel(posx(up),vel=VELOCITY,acc=ACC)
    def move_place(up,down):
        movel(posx(up),vel=VELOCITY,acc=ACC)
        movel(posx(down),vel=VELOCITY,acc=ACC)
        release()
        movel(posx(up),vel=VELOCITY,acc=ACC)
    def upside_down():
        release()
        movej(JReady, vel=VELOCITY, acc=ACC)
        movej(six_up_j, vel=VELOCITY, acc=ACC)
        movej(six_j, vel=VELOCITY, acc=ACC)
        grip()
        movel(posx([0,0,100,0,0,0]),vel=VELOCITY,acc=ACC, mod=DR_MV_MOD_REL)
        movej(posj([0,0,0,0,0,180]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        movel(posx([0,0,-95,0,0,0]),vel=VELOCITY,acc=ACC, mod=DR_MV_MOD_REL)
        release()
        movel(posx([0,0,95,0,0,0]),vel=VELOCITY,acc=ACC, mod=DR_MV_MOD_REL)
        movej(JReady, vel=VELOCITY, acc=ACC)
    set_tcp("2FG_TCP")
    set_tool("Tool Weight_2FG")
    while rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        for i in range(9):
            grip()
            move_up=tray1[i][:]
            move_up[2]=70
            movel(posx(move_up),vel=VELOCITY,acc=ACC)
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=3):
                pass
            release_compliance_ctrl()
            release()
            xyz,sol=get_current_posx()
            tray1_z[i]=xyz[2]
            movel(posx(move_up),vel=VELOCITY,acc=ACC)
        tray2_z = sorted(tray1_z)
        tray2_index = {value: index for index, value in enumerate(tray2_z)}
        moved_positions = [tray2_index[value] for value in tray1_z]
        release()
        for i in range(9):
            if moved_positions[i]==6:
                rest=i
                continue
            move_up=tray1[i][:]
            move_up[2]=100
            pick(move_up,tray1[i])
            move_up2=tray2[6][:]
            move_up2[2]=100
            move_place(move_up2,tray2[6])
            upside_down()
            pick(move_up2,tray2[6])
            move_up3=tray2[moved_positions[i]][:]
            move_up3[2]=100
            move_place(move_up3,tray2[moved_positions[i]])
        move_up=tray1[rest][:]
        move_up[2]=100
        pick(move_up,tray1[rest])
        move_up2=tray2[6][:]
        move_up2[2]=100
        move_place(move_up2,tray2[6])
        upside_down()
        
    rclpy.shutdown()
if __name__=="__main__":
    main()