# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

goal_status = [[0, 0, 0],
               [0, 0, 0],
               [0, 0, 0]]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("day2_1", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            set_tool,
            set_tcp,
            set_digital_output,
            movej,
            movel,
            wait,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    

    JReady = [0, 0, 90, 0, 90, 0]
    position_target_dic = {
        0: posx([346.473, 149.186, 23.457, 50.584, -178.66, 50.364]),
        1: posx([398.3905, 148.2835, 23.6555, 47.8575, -178.4625, 47.7795]),
        2: posx([450.308, 147.381, 23.854, 45.131, -178.265, 45.195]),
        3: posx([346.895, 98.171, 23.4805, 51.7335, -178.062, 51.5365]),
        4: posx([398.452, 97.4625, 23.621, 49.88725, -177.995, 49.83425]),
        5: posx([450.009, 96.754, 23.7615, 48.041, -177.928, 48.132]),
        6: posx([347.317, 47.156, 23.504, 52.883, -177.464, 52.709]),
        7: posx([398.5135, 46.6415, 23.5865, 51.917, -177.5275, 51.889]),
        8: posx([449.71, 46.127, 23.669, 50.951, -177.591, 51.069]),
    } 
    position_target_up_dic = {
        0: posx([346.473, 149.186, 100, 50.584, -178.66, 50.364]),
        1: posx([398.3905, 148.2835, 100, 47.8575, -178.4625, 47.7795]),
        2: posx([450.308, 147.381, 100, 45.131, -178.265, 45.195]),
        3: posx([346.895, 98.171, 100, 51.7335, -178.062, 51.5365]),
        4: posx([398.452, 97.4625, 100, 49.88725, -177.995, 49.83425]),
        5: posx([450.009, 96.754, 100, 48.041, -177.928, 48.132]),
        6: posx([347.317, 47.156, 100, 52.883, -177.464, 52.709]),
        7: posx([398.5135, 46.6415, 100, 51.917, -177.5275, 51.889]),
        8: posx([449.71, 46.127, 100, 50.951, -177.591, 51.069]),
    }
    position_goal_dic = {
        0: posx([346.175, -52.307, 22.925, 58.634, -177.364, 58.626]),
        1: posx([397.532, -52.514, 22.9275, 59.825, -177.339, 59.7315]),
        2: posx([448.889, -52.721, 22.93, 61.016, -177.314, 60.837]),
        3: posx([345.8875, -103.7, 22.9265, 63.6785, -177.3675, 63.485]),
        4: posx([397.30425, -103.8385, 23.004, 63.72525, -177.338, 63.54875]),
        5: posx([448.721, -103.977, 23.0815, 63.772, -177.3085, 63.6125]),
        6: posx([345.6, -155.093, 22.928, 68.723, -177.371, 68.344]),
        7: posx([397.0765, -155.163, 23.0805, 67.6255, -177.337, 67.366]),
        8: posx([448.553, -155.233, 23.233, 66.528, -177.303, 66.388]),
    }
    position_goal_up_dic = {
        0: posx([346.175, -52.307, 100, 58.634, -177.364, 58.626]),
        1: posx([397.532, -52.514, 100, 59.825, -177.339, 59.7315]),
        2: posx([448.889, -52.721, 100, 61.016, -177.314, 60.837]),
        3: posx([345.8875, -103.7, 100, 63.6785, -177.3675, 63.485]),
        4: posx([397.30425, -103.8385, 100, 63.72525, -177.338, 63.54875]),
        5: posx([448.721, -103.977, 100, 63.772, -177.3085, 63.6125]),
        6: posx([345.6, -155.093, 100, 68.723, -177.371, 68.344]),
        7: posx([397.0765, -155.163, 100, 67.6255, -177.337, 67.366]),
        8: posx([448.553, -155.233, 100, 66.528, -177.303, 66.388]),
    }


    # grip 관련 함수
    ON, OFF = 1, 0

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(0.5)

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        print("move first position")
        release()
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        j = 0    
        for i in range(9):
            # target 위로 이동
            print("movel target_up")
            movel(position_target_up_dic[i], vel=VELOCITY, acc=ACC)
            
            grip()

            # 내려가면서 힘 측정
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=3):
                pass
            release_compliance_ctrl()

            release()

            now_pos, sol = get_current_posx()
            if now_pos[2] < 50:
                for k in range(3):
                    if goal_status[k][0] == 0:
                        j = k
                        goal_status[k][0] = 1
                        break
                    else:
                        pass

            elif 50 <= now_pos[2] < 60:
                for k in range(3):
                    if goal_status[k][1] == 0:
                        j = k + 3
                        goal_status[k][1] = 1
                        break
                    else:
                        pass

            elif 60 <= now_pos[2]:
                for k in range(3):
                    if goal_status[k][2] == 0:
                        j = k + 6
                        goal_status[k][2] = 1
                        break
                    else:
                        pass


            print("movel target")
            movel(position_target_dic[i], vel=VELOCITY, acc=ACC)
            grip()

            print("movel target up")
            movel(position_target_up_dic[i], vel=VELOCITY, acc=ACC)

            print("movel goal up")
            movel(position_goal_up_dic[j], vel=VELOCITY, acc=ACC)

            print("movel goal")
            movel(position_goal_dic[j], vel=VELOCITY, acc=ACC)
            release()

            print("movel goal up")
            movel(position_goal_up_dic[j], vel=VELOCITY, acc=ACC)

    rclpy.shutdown()
if __name__ == "__main__":
    main()
