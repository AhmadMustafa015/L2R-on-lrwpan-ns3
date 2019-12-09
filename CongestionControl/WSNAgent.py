#!/usr/bin/env python3

import gym
import tensorflow as tf
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from tensorflow import keras
from ns3gym import ns3env
import random

env = gym.make('ns3-v0', port=5555)
ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.n)

s_size = ob_space.shape[0]
a_size = ac_space.n
print(a_size)
model = keras.Sequential()
model.add(keras.layers.Dense(s_size, input_shape=(s_size,), activation='relu'))
#model.add(keras.layers.Dense(24, activation='relu'))
model.add(keras.layers.Dense(a_size, activation='softmax'))
model.compile(optimizer=tf.compat.v1.train.AdamOptimizer(0.001),    # 'adam'?
              loss='categorical_crossentropy',
              metrics=['accuracy'])
model.summary()

total_episodes = 100    # env resets after episode
max_env_steps = 100     # max steps inside an episode
env._max_episode_steps = max_env_steps

epsilon = 1.0               # exploration rate
epsilon_min = 0.01
epsilon_decay = 0.999       # experiment

time_history = []
rew_history = []
state = env.reset()
for e in range(total_episodes):
    
    print("new episode")
    
    print("reach?")
    state = np.reshape(state, [1, s_size])
    rewardsum = 0
    for time in range(max_env_steps):
        # Choose action
        if np.random.rand(1) < epsilon:
            action = np.random.randint(a_size)

        else:
            action = np.argmax(model.predict(state)[0]) # ???

        
        # Step
        print("---action: ", action)
        next_state, reward, done, info = env.step(action)
        print("---obs, reward, done, info: ", next_state, reward, done, info)

        if done:
            print("episode: {}/{}, time: {}, rew: {}, eps: {:.2}"
                  .format(e, total_episodes, time, rewardsum, epsilon))
            break
        
        next_state = np.reshape(next_state, [1, s_size])

        # Train
        target = reward # ??
        if not done:
            target = (reward + 0.95 * np.amax(model.predict(next_state)[0])) # experiment
            print(target)

        target_f = model.predict(state)
        #print(target_f)
        #print(target_f[0][action])
        target_f[0][action] = target
        model.fit(state, target_f, epochs=1, verbose=0)

        state = next_state
        rewardsum += reward
        if epsilon > epsilon_min: epsilon *= epsilon_decay
    print("episode passed")
    print(e)
    time_history.append(time)
    rew_history.append(rewardsum)
    
print("Plot Learning Performance")
mpl.rcdefaults()
mpl.rcParams.update({'font.size': 16})

fig, ax = plt.subplots(figsize=(10,4))
plt.grid(True, linestyle='--')
plt.title('Learning Performance')
plt.plot(range(len(time_history)), time_history, label='Steps', marker="^", linestyle=":")#, color='red')
plt.plot(range(len(rew_history)), rew_history, label='Reward', marker="", linestyle="-")#, color='k')
plt.xlabel('Episode')
plt.ylabel('Time')
plt.legend(prop={'size': 12})

plt.savefig('learning.pdf', bbox_inches='tight')
plt.show()