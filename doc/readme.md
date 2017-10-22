

in previous projects, the path is simply given as a polynomial, the controller does the all the control work, the acceleration/deacceleration, steering etc. 

in this lecture, I will implement a more sophisticated path, the path is given as a list of waypoints, in cartisian coordinates. 

in the esscense, this is a search problem. the technique given in the lectures are the hybrid A-star method, for continuous space. 

to do this, I need the fusion data of the other veichles, which is given in the project, by the simulator, in the form of x, y coordinates; i also need to know the the lane boundaries, which is given as a map information. 

there will be many possible solutions, a cost function is given to make the optimal one. how to define and use the cost function is a problem to solve. 

the path planning includes three sub-components: 
1. the prediction, which predicts the other viechile's dynamic path;
2. the behaviour planning, which gives instructions on how to monevour the car, the result is keep lane, change lane right, change lane left, prepare change lane left/right, etc, it's implmented as a infinite state machine.
3. the last comonent is the trajectory planning, which solves the search problem using the A-star for continuous space. 

