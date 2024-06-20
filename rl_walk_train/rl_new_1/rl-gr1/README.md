training code redme


1. pip install legged-gym and rsl-rl
2. in rl-gr1/legged_gym/legged_gym/scripts  run "python train.py --task=GR1T1 --headless"
3. reward function and config in "rl-gr1/legged_gym/legged_gym/envs/GR1T1"
4. "rl-gr1/legged_gym/legged_gym/envs/GR1T1/gr1t1_config_lstm.py" is RL walk
"rl-gr1/legged_gym/legged_gym/envs/GR1T1/gr1t1_config_lstm_stand.py" is RL stand
"rl-gr1/legged_gym/legged_gym/envs/GR1T1/gr1t1_config_lstm_student.py" is RL student, train a encoder to fit base linear velocity and height
if you want use student , you need copy best policy to "rl-gr1/legged_gym/resources/teacher_nets", and edit path in "rl-gr1/legged_gym/legged_gym/envs/GR1T1/gr1t1_config_lstm_student.py"
5. train different task need chenge config in "rl-gr1/legged_gym/legged_gym/envs/__init__.py"

6. the motor transfer function defined in "rl-gr1/legged_gym/legged_gym/envs/base/lstm_model.py"
7. "rl-gr1/legged_gym/logs/GR1T1/_May31_19-49-11"  is a walk policy, you can train resume