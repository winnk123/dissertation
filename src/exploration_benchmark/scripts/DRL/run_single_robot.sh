#!/bin/sh
env="Gazebo"
num_agents=1
algo="mappo"
exp="robot1"

echo "env is ${env}, algo is ${algo}, exp is ${exp}"
CUDA_VISIBLE_DEVICES=0 python single_robot_1.py \
--env_name ${env} --algorithm_name ${algo} --experiment_name ${exp} \
--user_name "xyf" --num_agents ${num_agents} \
--cnn_layers_params '16,3,1,1 32,3,1,1 16,3,1,1' --hidden_size 64 \
--use_merge --use_recurrent_policy
