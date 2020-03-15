import simulation as s
path=open('path.txt').read()
sim = s.Simulation(2)
sim.start()
# [0.5, 1, 1.5, 2], [.5,8,1.5,9] 
obs = [[[.5, 1, 0], [1.5, 2, .5]], [[.5, 8, 0], [1.5, 9, .5]]]
sim.createObstacles(obs)
sim.createGoals([[0, 0.5, 0.3]])
sim.readPlan(path, " ", False)

