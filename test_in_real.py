import argparse
import stable_baselines3
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
import os
from rl_env_side import Sim

import torch
torch.cuda.is_available = lambda : False
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
ENVIROMENT_CLASSES = {
    'real': Sim
}


def make_env(env_class_name):
    env_class = ENVIROMENT_CLASSES[env_class_name]
    return Monitor(env_class())


def main():
    parser = argparse.ArgumentParser(description='Gripper DRL.')
    parser.add_argument('--jobs', default=1, type=int,
                        help='Number of parallel simulations')
    parser.add_argument('--algorithm', default='SAC',
                        type=str, help='Algorithm')
    #add trained model ./home/zj/Documents/GitHub/RL_ICRA2023/logs/models/rl_model_10_steps.zip at default option
    parser.add_argument('--model', default='', type=str,
                        help='Path to the model')
    parser.add_argument('--play-only', default=False, action='store_true')
    parser.add_argument('--environment', default='real', type=str)
    parser.add_argument('--lr', default=0.003, type=float)

    # sac
    parser.add_argument('--gradient_steps', default=1, type=int)
    parser.add_argument('--train_freq', default=1, type=int)
    parser.add_argument('--buffer_size', default=10000, type=int)
    parser.add_argument('--learning_starts', default=10, type=int)
    parser.add_argument('--gamma', default=0.99, type=float)
    parser.add_argument('--batch_size', default=64, type=int)
    args = parser.parse_args()

    env = make_env(args.environment)
    # Load DRL algorithm
    drl_algorithm_classes = {
        'SAC': stable_baselines3.SAC
    }
    drl_algorithm_class = drl_algorithm_classes[args.algorithm]

    # Initialize model
    if args.model != '':
        # Load model
        model = drl_algorithm_class.load(
            args.model, env=env, learning_rate=args.lr)
        print('load model successful')
    else:
        algorithm_args = {}

        policy = 'MlpPolicy'

        if args.algorithm == 'SAC':
            algorithm_args['buffer_size'] = args.buffer_size
            algorithm_args['gradient_steps'] = args.gradient_steps
            algorithm_args['train_freq'] = (1, "step")
            algorithm_args['learning_starts'] = args.learning_starts
            algorithm_args['gamma'] = args.gamma
            algorithm_args['batch_size'] = args.batch_size

        # Model from scratch
        model = drl_algorithm_class(
            policy,
            env,
            device='cpu',
            learning_rate=args.lr,
            verbose=1,
            tensorboard_log='logs/tb/flip',
            **algorithm_args
        )

    # Play or learn
    if args.play_only:
        # Play
        number_of_episodes = 0
        number_of_successes = 0
        obs = env.reset()
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            print(action)
            if done:
                # break
                number_of_episodes += 1
                if reward >= 10:
                    number_of_successes += 1
                print(
                    f'Success picks {number_of_successes}/{number_of_episodes} ({(number_of_successes/number_of_episodes) * 100}%)')
                obs = env.reset()
    else:
        # Learn
        model.learn(
            total_timesteps=100_000_000,
            callback=[
                CheckpointCallback(
                    save_freq=1,
                    save_path='./logs/models/flip',
                ),
                # CustomEvalCallback(env, target_number_of_objects=args.target_number_of_objects,
                # workspace_radius=args.workspace_radius),
            ]
        )
        model.save('final')


if __name__ == '__main__':
    main()
