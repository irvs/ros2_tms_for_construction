#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Yuichiro Kasahara
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
Define sample_zx200_navigate_anywhere.

Created on Mon Nov 03 2025
@author: Yuichiro Kasahara
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from tms_flexbe_states.Excavator.excavator_navigate_anywhere import ExcavatorNavigateAnywhere

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class sample_zx200_navigate_anywhereSM(Behavior):
    """
    Define sample_zx200_navigate_anywhere.

    This task is sample implemented on FlexBE. This task contains only one node crresponing to "ExcavatorNavigateAnywhere".
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'sample_zx200_navigate_anywhere'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        ExcavatorNavigateAnywhere.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        with _state_machine:
            # x:269 y:67
            OperatableStateMachine.add('ExcavatorNavigateAnywhere',
                                       ExcavatorNavigateAnywhere(model_name="ic120", record_name="SampleNavigateToPose"),
                                       transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
                                       autonomy={'received': Autonomy.Off, 'aborted': Autonomy.Off, 'no_connection': Autonomy.Off, 'data_error': Autonomy.Off},
                                       remapping={'data': 'data'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
