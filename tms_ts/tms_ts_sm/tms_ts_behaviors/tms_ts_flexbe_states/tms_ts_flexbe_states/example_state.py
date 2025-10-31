#!/usr/bin/env python

# Copyright 2023 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration state."""
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration

from flexbe_core import EventState, Logger


class ExampleState(EventState):
    """
    Example for a state to demonstrate which functionality is available for state implementation.

    All FlexBE states should inherit from EventState.

    This example lets the behavior wait until the given target_time has passed since
    entering the state.

    The state also records the time the behavior was activated and exited.

    The UI parses this description for data about the state to diplay.

    List parameter values with double hyphens
    -- target_time     float     Time which needs to have passed since the behavior started.

    List labeled outcomes using the double arrow notation (must match constructor)
    <= done            Given time has passed.
    <= failed          Example for a failure outcome.

    List input and output user data that is passes along using
        These are optional and not included in this example.
    """

    def __init__(self, target_time):
        """Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments."""
        super().__init__(outcomes=['done', 'failed'])

        # Store state parameter for later use.
        self._target_wait_time = Duration(seconds=target_time)

        # The constructor is called when building the state machine, not when actually starting the behavior.
        # Thus, we cannot save the starting time now and will do so later.
        self._state_start_time = None
        self._state_enter_time = None
        self._state_exit_time = None
        self._return = None
        self._elapsed_time = Duration(nanoseconds=2**63 - 1)

    @property
    def elapsed_seconds(self):
        """Log elapsed time since start as simple string."""
        return f"{self._elapsed_time.nanoseconds / S_TO_NS:.3f}"

    @property
    def target_seconds(self):
        """Log target wait time as a simple string."""
        return f"{self._target_wait_time.nanoseconds / S_TO_NS:.3f}"

    @property
    def start_time(self):
        """Log state start time (NOT enter time!) as a simple string."""
        return f"{self._state_start_time.nanoseconds / S_TO_NS:.3f}"

    @property
    def enter_time(self):
        """Log state enter time as a simple string."""
        return f"{self._state_enter_time.nanoseconds / S_TO_NS:.3f}"

    @property
    def exit_time(self):
        """Log state exit time as a simple string."""
        return f"{self._state_exit_time.nanoseconds / S_TO_NS:.3f}"

    @property
    def clock_time(self):
        """Log system time in 1 hour increments using simple string."""
        time_msg = self._node.get_clock().now().to_msg()
        time = time_msg.sec % 3600 + time_msg.nanosec / S_TO_NS
        return f"{time:.3f}"

    # Standard methods of EventState
    # Normally we override on_enter, execute, and on_exit.
    # Also demonstrating on_start and on_stop here
    # This example includes more logging than normally used for demonstration.
    def execute(self, userdata):
        """
        Execute this method periodically while the state is active.

        Main purpose is to check state conditions and trigger a corresponding outcome.
        If no outcome is returned, the state will stay active.
        """
        if self._return is not None:
            # We must be blocked by autonomy level.
            # Here we will just return the prior outcome and not recalculate

            # Local info is NOT sent to the UI, and only shown in logs and terminal
            Logger.localinfo(f"execute blocked for '{self._name}' state ({self.path}) @ {self.clock_time} "
                             f"- use prior return code={self._return}")
            return self._return

        # Normal calculation block
        try:
            self._elapsed_time = ExampleState._node.get_clock().now() - self._state_enter_time
            if self._elapsed_time >= self._target_wait_time:
                Logger.loginfo(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                               f"- done waiting at {self.elapsed_seconds} seconds.")
                self._return = 'done'
                return 'done'  # One of the outcomes declared above.
        except Exception:  # pylint:disable=W0703
            # Something went wrong
            Logger.logerr(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                          f"- something went wrong after {self.elapsed_seconds} seconds.")
            self._return = 'failed'
            return 'failed'

        # Local info is NOT sent to the UI, and only shown in logs and terminal
        Logger.localinfo(f"execute for '{self._name}' state ({self.path}) @ {self.clock_time} "
                         f"- {self.elapsed_seconds} seconds since start.")
        return None  # This is normal behavior for state to continue executing

    def on_enter(self, userdata):
        """
        Call this method when the state becomes active.

        That is, when a transition from another state to this one is taken.
        It is primarily used to start actions which are associated with this state.

        The following code is just for illustrating how the behavior logger works.
        Text logged by the behavior logger is sent to the operator and displayed in the GUI.
        """
        self._state_enter_time = ExampleState._node.get_clock().now()
        self._elapsed_time = Duration(seconds=0.0)
        self._return = None  # Clear return code on entry

        Logger.loginfo(f"on_enter for '{self._name}' state ({self.path}) @ {self.clock_time} "
                       f"- need to wait for {self.target_seconds} seconds.")

    def on_exit(self, userdata):
        """
        Call this method when an outcome is returned and another state gets active.

        It can be used to stop possibly running processes started by on_enter.
        Nothing to do in this example.
        """
        self._state_exit_time = ExampleState._node.get_clock().now()
        Logger.loginfo(f"on_exit for '{self._name}' state ({self.path}) @ {self.clock_time} "
                       f"elapsed time = {self.elapsed_seconds} seconds.")

    def on_start(self):
        """
        Call this method when the behavior is instantiated on board.

        If possible, it is generally better to initialize used resources in the constructor
        because if anything failed, the behavior would not even be started.
        In this example, we use this event to set the correct start time.
        """
        self._state_start_time = ExampleState._node.get_clock().now()
        Logger.loginfo(f"on_start for '{self._name}' state ({self.path}) @ {self.start_time} seconds "
                       f" time to wait = {self.target_seconds} seconds..")

    def on_stop(self):
        """
        Call this method whenever the behavior stops execution, also if it is cancelled.

        Use this event to clean up things like claimed resources.
        Nothing to do in this example.
        """
        self._elapsed_time = ExampleState._node.get_clock().now() - self._state_start_time
        Logger.loginfo(f"on_stop for '{self._name}' state ({self.path}) @ {self.clock_time} seconds "
                       f" total behavior instance elapsed time = {self.elapsed_seconds} seconds ")
        if self._state_enter_time is None:
            Logger.loginfo(f"on_stop for '{self._name}' state ({self.path}) @ {self.clock_time} seconds "
                           f" - never entered the state to execute! ")
        else:
            try:
                self._elapsed_time = self._state_exit_time - self._state_enter_time
                Logger.loginfo(f"    '{self._name}' state "
                               f"was active (enter-to-exit) for {self.elapsed_seconds} seconds.")
            except Exception:  # pylint: disable=W0703
                Logger.logerr(f"  entered at time={self.enter_time} seconds but never exited!")
