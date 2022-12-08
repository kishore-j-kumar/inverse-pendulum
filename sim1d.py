import gym

env = gym.make('InvertedPendulum-v4', render_mode="human")
env.action_space.seed(42)

observation, info = env.reset(seed=42)
last_angles = []

for _ in range(10000):
    action = env.action_space.sample()
    print("Action: ", action)
    observation, reward, terminated, truncated, info = env.step(action)
    last_angles.append(observation[2])
    if len(last_angles) > 10 and sum(last_angles[-10:-1])/10 < 0:
        action = [1]
    elif len(last_angles) > 10 and sum(last_angles[-10:-1])/10 > 0:
        action = [0]
    elif observation[2] < 0:
        action = [1]
    else:
        action = [0]
    print(observation, reward, terminated, truncated, info)
    if terminated or truncated:
        observation, info = env.reset()

env.close()