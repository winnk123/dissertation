#!/bin/sh
env="GridEnv"
num_agents=2
num_obstacles=0
algo="mappo"
exp="grid_training7"
seed_max=1

echo "env is ${env}, scenario is ${scenario}, algo is ${algo}, exp is ${exp}, max seed is ${seed_max}"
for seed in `seq ${seed_max}`;
do
    echo "seed is ${seed}:"
    CUDA_VISIBLE_DEVICES=0 python train/train_grid.py \
    --env_name ${env} --algorithm_name ${algo} --experiment_name ${exp} \
    --log_interval 1 --save_interval 1 --use_wandb --wandb_name "mapping" --user_name "xyf" --num_agents ${num_agents}\
    --num_obstacles ${num_obstacles} --cnn_layers_params '16,3,1,1 32,3,1,1 16,3,1,1' --hidden_size 64 --seed 1 --n_training_threads 1 \
    --n_rollout_threads 2 --num_mini_batch 1 --num_env_steps 500000 --ppo_epoch 3 --gain 0.01 \
    --lr 1e-3 --critic_lr 1e-3 --max_steps 20 --use_complete_reward --use_intrinsic_reward --use_eval --n_eval_rollout_threads 1 \
    --agent_view_size 7 --local_step_num 1 --use_random_pos --use_same_location --use_merge --use_recurrent_policy \

done

