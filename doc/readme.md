# Path Planning Project Documentation 

## Understanding of Path Planning Modules
The Path Planning module includes 4 separate sub-modules: Search, Predication, Behavior planning and Trajectory generation. 

In the Search module, Sebastian explains two different search methods in a discrete context, the cell-expansion method and A-star method. Both methods requires the context is divided into cells, the cell-expansion method expand the search into every possible cell; A-star method uses an optimized heuristic matrix to aid the search, this gains performance. Besides this difference, both has it's own advantages and disadvantages; 

The predication module introduces how to predict the behavior of OTHER dynamic objects in the environment. Two methods were introduced, one is the model based, and another one is data driven. Again, both have their own pros and cons, but good news is they can be used together. 

The Behavior planning module explains how "our car" respond to the context. The behavior module acts like a navigator sitting beside the driver, telling the driver when and where to change lane, to stop, to follow a leading car with certain distance in between, etc. But it doesn't give instructions on how to perform it, they are performed by the controller module. One possible tools the behavior module uses is the infinite state machine, it predefines many possible states which the car can transfer into and from. 

The Trajectory plan module is the "brain" of the self-driving car; it uses information from behavior planning, localization, sensor fusion data, map information to generate a series of waypoints, which the car can follow. The methods introduced in the Search module is practical in an unstructured environment, like parking slots, and relevant for robots which has no angel constraints; while for a car driving in a highway, things are different, first, due to the steering angel constraints, the path generated must by drivable, second, the highway, unlike the parking slots, are more structural, like the boundaries, and these factors needs to be addressed. To solve the drivable problem, a hybrid A-star method is introduced, which takes the car's steering angel into account when searching path; to take into account the structural nature, a Polynomial Trajectory Generation (PTG) method is introduced, which generates multiple possible path the car can follow, the optimized path is determined by applying the cost function to them, and the one with the minimum cost is chosen. The cost function can be defined based on various factors, for example, the jerk value, speed of limit, the position in the lane etc. 

Unlike previous courses, there is a code walk-through in the project module. The video provides a very good start on the project, it gives a pipeline on how to complete the project, my project is based on it, and details are explained in following section; 


## The model used in this project
I followed the idea presented in the walkthrough video, which is using spline package to generate the trajectory path. 

The video gives detailed explanation and solution on various aspects need to consider in the highway driving, here I gave them an outline. More detailed explanation and my understanding are given in the code. 

### Drive with speed limit

The highway has a speed limit of 50 mph, to address this, I set the speed limit to 49.5. 


### Jerk avoidance
To avoid jerk, the car's speed is incrementally increased to the maximum speed.

### Lane Changing
To change lane, a state machine is used. the details are as follows: 

Once the car is too close to the leading car, then prepare the lane change. 

1. align the cars by each lane;
2. if the car is in the middle lane, first check if the lane is clear or safe to make the left lane change; if true, make left_lane_change as true, and set the lane to 0; if not true, check if the right lane is clear or safe to change, of true, set the right_lane_change to true, and lane to 2; if either side is unsafe, then the car stays in the current lane and slow down to keep a safe distance. 
3. if the car is in the either side of lane, lane 0 or lane 2, then check if the middle lane is clear or safe to make the change, if true, set the middle_lane_change is true and lane to 1; if not, the car stays in the current lane and slow down for safe distance. 

The implementation details are explained in the code. Run the code with the simulator, seems that all the rubric points are met. 


## Future work
The project is working, but not the perfect one. 
==todo::



in previous projects, the path is simply given as a polynomial, the controller does the all the control work, the acceleration/deacceleration, steering etc. 

in this lecture, I will implement a more sophisticated path, the path is given as a list of waypoints, in cartisian coordinates. 

in the esscense, this is a search problem. the technique given in the lectures are the hybrid A-star method, for continuous space. 

to do this, I need the fusion data of the other veichles, which is given in the project, by the simulator, in the form of x, y coordinates; i also need to know the the lane boundaries, which is given as a map information. 

there will be many possible solutions, a cost function is given to make the optimal one. how to define and use the cost function is a problem to solve. 

the path planning includes three sub-components: 
1. the prediction, which predicts the other viechile's dynamic path;
2. the behaviour planning, which gives instructions on how to monevour the car, the result is keep lane, change lane right, change lane left, prepare change lane left/right, etc, it's implmented as a infinite state machine.
3. the last comonent is the trajectory planning, which solves the search problem using the A-star for continuous space. 

how to choose the reasonable trajetorr is dertermined by the cost function. there are various cost function considering different factors, the cost function is one area needs to be tweakd. 



