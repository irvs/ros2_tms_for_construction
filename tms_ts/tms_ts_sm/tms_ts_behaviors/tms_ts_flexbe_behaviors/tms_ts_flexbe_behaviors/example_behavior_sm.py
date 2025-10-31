#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2015 Philipp Schillinger
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

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Example Behavior.

Created on Fri Aug 21 2015
@author: Philipp Schillinger
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class ExampleBehaviorSM(Behavior):
    """
    Define Example Behavior.

    This is a simple example for a behavior.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Example Behavior'

        # parameters of this behavior
        self.add_parameter('waiting_time', 3)

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        LogState.initialize_ros(node)
        WaitState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        log_msg = "Hello World!"
        # x:83 y:390
        _state_machine = OperatableStateMachine(outcomes=['finished'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:52 y:78
            OperatableStateMachine.add('Print_Message',
                                       LogState(text=log_msg, severity=Logger.REPORT_HINT),
                                       transitions={'done': 'Wait_After_Logging'},
                                       autonomy={'done': Autonomy.Low})

            # x:40 y:228
            OperatableStateMachine.add('Wait_After_Logging',
                                       WaitState(wait_time=self.waiting_time),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
