

class TestSubtaskMove():
    def __init__(self):
        self.subtask_client = ActionClient(self, TsDoSubtask, "subtask_node_9001", callback_group=ReentrantCallbackGroup())

        print(self.subtask_client)

        goal_msg = TsDoSubtask.Goal()
        goal_msg.arg_json = '{"position": [0.0, 1.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}'

        self.goal_handle = await self.subtask_client.send_goal_async(goal_msg)
        if not self.goal_handle.accepted:
            self.get_logger().info("goal rejected")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Rejected"
            return result

        self.get_logger().info("goal accept")
        self.future_result = await self.goal_handle.get_result_async()
        result = TsDoTask.Result()
        #if self.future_result.done():
        if True:
            self.get_logger().info(self.future_result.result.message)
            goal_handle.succeed()
            #result.message = self.future_result.result.message
            result.message = "Success"
        else:
            goal_handle.abort()
            result.message = self.future_result.result.message