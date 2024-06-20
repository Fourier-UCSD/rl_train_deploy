# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR
from .base.legged_robot import LeggedRobot
from legged_gym.utils.task_registry import task_registry

# from .GR1.gr1 import GR1
# from .GR1.gr1_config_0416 import GR1Cfg, GR1CfgPPO

from .GR1.gr1_lstm import GR1
from .GR1.gr1_config_lstm import GR1Cfg, GR1CfgPPO

# from .GR1L.gr1l import GR1L
# from .GR1L.gr1l_config_random import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_stand import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_walk import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_run import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_walk_step1 import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_walk_step2 import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_config_jump_step1 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm import GR1L
# from .GR1L.gr1l_config_lstm import GR1LCfg, GR1LCfgPPO


# from .GR1L.gr1l_lstm_start2 import GR1L
# from .GR1L.gr1l_config_lstm_start_2 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm_2 import GR1L
# from .GR1L.gr1l_config_lstm_2 import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_lstm_start_3 import GR1L
# from .GR1L.gr1l_config_lstm_start_3 import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_code_moe import GR1L
# from .GR1L.gr1l_config_moe import GR1LCfg, GR1LCfgPPO
# from .GR1L.gr1l_code_option2 import GR1L
# from .GR1L.gr1l_config_option2 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm_start_3 import GR1L
# from .GR1L.gr1l_config_lstm_start_3 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm_3  import GR1L
# from .GR1L.gr1l_config_lstm_3 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm_3_2  import GR1L
# from .GR1L.gr1l_config_lstm_3_2 import GR1LCfg, GR1LCfgPPO

# from .GR1L.gr1l_lstm_start_4 import GR1L
# from .GR1L.gr1l_config_lstm_start_4 import GR1LCfg, GR1LCfgPPO

from .GR1L.gr1l_lstm_start_5_3 import GR1L
from .GR1L.gr1l_config_lstm_start_5_3 import GR1LCfg, GR1LCfgPPO

from .GR1LowerBody.gr1lowerbody import GR1LowerBody
from .GR1LowerBody.gr1lowerbody_config import GR1LowerBodyCfg, GR1LowerBodyCfgPPO

# from .GR1LowerBody.gr1lowerbody_config_stand import GR1LowerBodyCfg, GR1LowerBodyCfgPPO
# from .GR1LowerBody.gr1lowerbody_config_walk import GR1LowerBodyCfg, GR1LowerBodyCfgPPO
# from .GR1LowerBody.gr1lowerbody_config_step1 import GR1LowerBodyCfg, GR1LowerBodyCfgPPO
# from .GR1LowerBody.gr1lowerbody_config_run import GR1LowerBodyCfg, GR1LowerBodyCfgPPO
# from .GR1T1.gr1t1_config import GR1T1Cfg, GR1T1CfgPPO
# from .GR1T1.gr1T1 import GR1T1

# from .GR1T1.gr1t1_config_lstm_student import GR1T1Cfg, GR1T1CfgPPO
from .GR1T1.gr1t1_config_lstm_stand import GR1T1Cfg, GR1T1CfgPPO

# from .GR1T1.gr1t1_config_lstm import GR1T1Cfg, GR1T1CfgPPO
from .GR1T1.gr1t1_lstm import GR1T1

# from .GR1T1.gr1t1_config_lstm_rxy import GR1T1Cfg, GR1T1CfgPPO
# from .GR1T1.gr1t1_lstm import GR1T1

task_registry.register("GR1", GR1, GR1Cfg(), GR1CfgPPO())
task_registry.register("GR1L", GR1L, GR1LCfg(), GR1LCfgPPO())
task_registry.register("GR1LowerBody", GR1LowerBody, GR1LowerBodyCfg(), GR1LowerBodyCfgPPO())
task_registry.register("GR1L_new", GR1L, GR1LCfg(), GR1LCfgPPO())
task_registry.register("GR1T1", GR1T1, GR1T1Cfg(), GR1T1CfgPPO())
