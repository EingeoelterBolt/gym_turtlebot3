from gym.envs.registration import register

register(
    id='turtlebot3-v0',
    entry_point='gym_turtlebot3.envs:Turtlebot3Env',
)
