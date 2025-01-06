# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

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
            get_current_posj,
            set_tool,
            set_tcp,
            set_digital_output,
            get_digital_input,
            check_position_condition,
            movej,
            amovej,
            movel,
            wait,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_SSTOP,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    

    JReady = [0, 0, 90, 0, 90, 0]
    target =  posx([573.592, 145.82, 44.769, 54.976, -177.812, 54.943])
    target_up = posx([573.592, 145.82, 120, 54.976, -177.812, 54.943])
    goal =  posx([571.076, -153.513, 44.501, 59.365, -177.992, 59.455])
    goal_close = posx([571.076, -153.513, 75, 59.365, -177.992, 59.455])
    goal_up = posx([571.076, -153.513, 120, 59.365, -177.992, 59.455])

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
        
        # target 위로 이동
        print("movel target_up")
        movel(target_up, vel=VELOCITY, acc=ACC)
        
        # target으로 이동 후 집기
        print("movel target_up")
        movel(target, vel=VELOCITY, acc=ACC)
        grip()

        # target 위로 이동
        print("movel target_up")
        movel(target_up, vel=VELOCITY, acc=ACC)

        # goal 위로 이동
        print("movel target_up")
        movel(goal_up, vel=VELOCITY, acc=ACC)

        # goal 근처로 이동
        print("movel target_up")
        movel(goal_close, vel=VELOCITY, acc=ACC)

        # 내려가면서 힘 측정
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -5, 0, 0, 5], dir=[0, 0, 1, 0, 0, 1], mod=DR_FC_MOD_REL)
        while True:
            if check_position_condition(axis=DR_AXIS_Z, min=45, ref=DR_BASE,mod=DR_MV_MOD_ABS):
                break
        release_compliance_ctrl()
        
        release()

    rclpy.shutdown()
if __name__ == "__main__":
    main()



