"""
PRINT ONLY FOR ONE KEY = 10
TRUE/FALSE  terminal state or not.
"""
import gym
env = gym.make('FrozenLake-v1')
P = env.env.P
print(type(P))
#for key, value in P.items():
	#print (key, ": ", value)
for key, value in P.items():
	for k, v in value.items():
		if key == 10:
			print (key, ": ", k, " : ", v)

