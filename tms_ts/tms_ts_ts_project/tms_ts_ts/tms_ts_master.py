import time
import subprocess
import rclpy
from rclpy.node import Node
from tms_msg_ts.srv import TsReq
from tms_msg_ts.srv import TsStateControl


class Task:
    def __init__(self):
        self.rostime = 0
        self.task_id = 0
        self.robot_id = 0
        self.object_id = 0
        self.user_id = 0
        self.place_id = 0
        self.priority = 0
    
    def __lt__(self, other):
        return (self.priority, self.rostime) < (other.priority, other.rostime)


class TaskManagerElements:
    def __init__(self, task):
        self.num = 0
        self.flag = True
        self.task = task


class TmsTsMaster(Node):

    def __init__(self):
        super().__init__('tms_ts_master')
        self.state_condition = -1
        self.loop_counter = 0
        self.abort = False
        self.task_manager = []
        self.task_list = []

        # Callbacks(service)
        self.create_service(TsReq, 'tms_ts_master', self.tms_ts_master_callback)
        self.create_service(TsStateControl, 'ts_state_control', self.ts_state_control_callback) 

    def create_service_call(self, rostime, task_id, robot_id, object_id, user_id, place_id, priority, thread_num):
        print("[create_service_call] >> start")
        service_name = "request"

        command = (
            'ros2 service call /'
            + service_name
            + str(self.task_manager[thread_num].num)
            + ' "'
            + str(rostime)
            + '" "'
            + str(task_id)
            + '" "'
            + str(robot_id)
            + '" "'
            + str(user_id)
            + '" "'
            + str(place_id)
            + '" "'
            + str(priority)
            + '"\n'
        )
        return command

    def create_run_command(self, thread_num):
        command = (
            'ros2 run --help'  # TODO: write run command 
        )
        return command
    
    def execute_command(self, buf):
        print("[execute_command] >> " + buf)
        subprocess.run(buf.split())
        # TODO: check cannnot run the command 

        return True


    def tms_ts_master_callback(self, request, response):
        print("[tms_ts_master_callback] >> start")

        task = Task()
        task.task_id = request.task_id
        task.robot_id = request.robot_id
        task.object_id = request.object_id
        task.user_id = request.user_id
        task.place_id = request.place_id
        task.priority = request.priority
        task.rostime = int(time.time() * 1000000) #nsec

        self.task_list.append(task)
        self.task_list.sort()

        # print all tasks in task_list
        for task in self.task_list:
            print("task_id: " + str(task.task_id))
            print("priority: " + str(task.priority))

        response.result = 1
        return response

    def ts_state_control_callback(self, request, response):
        print("[ts_state_control_callback] >> start")
        if request.type == 0:  # judge segment(from TS)
            if request.cc_subtasks == 0:
                pass
            elif request.cc_subtasks >= 2:
                pass
            else:
                self.get_logger().info("Illegal subtasks number.")
                self.state_condition = -1
                self.loop_counter = 0
                response.result = 0
                return response
        elif request.type == 1:
            pass
        elif request.type == 2:
            pass
        else:
            pass
    
    def add_thread(self, thread_num, arg1, arg2):
        pass


def main(args=None):
    rclpy.init(args=args)

    tms_ts_master = TmsTsMaster()

    rclpy.spin(tms_ts_master)

    tms_ts_master.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



        

