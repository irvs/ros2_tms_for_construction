import rclpy
from rclpy.node import Node
from enum import IntEnum
from tms_msg_rp.srv import RpCmd

class UNIT(IntEnum):
    ALL = 0
    VEHICLE = 1
    ARM_R = 2
    ARM_L = 3
    GRIPPER_R = 4
    GRIPPER_L = 5
    LUMBA = 6
    CC = 7


class CMD(IntEnum):
    CLEARALARM = 0
    SETPOWER = 1
    SETSERVO = 2
    PAUSE = 3
    RESUME = 4
    ABORT = 5
    STOP = 6
    GETSTATE = 7
    SET_ODOM = 7
    GETPOSE = 8
    SYNC_OBJ = 8
    MOVE_TRAJECTORY = 8
    CALC_BACKGROUND = 9
    MOVE_ABS = 15
    MOVE_REL = 16


class SubtaskData:
    def __init__(self):
        self.type = False
        self.robot_id = 0
        self.arg_type = 0
        self.v_arg = []


class TmsTsSubtask(Node):

    def __init__(self):
        super().__init__('tms_ts_subtask')
        self.create_service(RpCmd, "rp_cmd", self.rp_cmd_srv)
    
    def rp_cmd_srv(self, request, response):
        print("[rp_cmd_srv] >> start")

        cmd = request.command
        if cmd == 9001:  # move
            print("[tms_rp] move command")
        elif cmd == 9002:  # grasp
            print("[tms_rp] grasp command")
        elif cmd == 9003:  # release
            print("[tms_rp] release command")
        elif cmd == 9004:  # open the refrigerator
            print("[tms_rp] open command")
        elif cmd == 9005:  # close the refrigerator
            print("[tms_rp] close command")
        elif cmd == 9006:  # kobuki_random_walker
            print("[tms_rp] random_move command")
        elif cmd == 9007:  # sensing using kinect
            print("[tms_rp] sensing command")
        else:  # No such subtask
            print("No such subtask (ID: %d)"%(cmd))

        print("[rp_cmd_srv] >> exit from subtask function")
        response.result = 1
        return response

    def radian_normalize(self, rad):
        pass
    
    def diff_radian(self, rad1, rad2):
        pass

    def distance(self, x1, y1, x2, y2):
        pass
    
    def send_rc_exception(self, error_type):
        pass
    
    def get_robot_pos(self, type, robot_id, robot_name, rp_srv):
        pass
    
    
    
    def sp5_control(self, type, unit, cmd, arg_size, arg):
        pass

    def _move(self, subtask_data):
        pass
    
    def _grasp(self, subtask_data):
        pass
    
    def _release(self, subtask_data):
        pass

def main(args=None):
    rclpy.init(args=args)

    tms_ts_subtask = TmsTsSubtask()

    rclpy.spin(tms_ts_subtask)

    tms_ts_subtask.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
