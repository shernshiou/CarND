# Reflection
## Describe the effect each of the P, I, D components had in your implementation.
Each P, I, and D has a significant effect when driving the car.
P, proportional control corrects the error correlate to CTE. From the simulator, we can see that the P tries to adjust the error by making a counter steer. Using the P, the car tries to stay to the center of the road.
High P tries to adjust the steer more which causes sharp steer.
Low P is less sensitive to the CTE which causes the turn to be less sharp.

I, integral control calculates the number of accumulated errors. The higher the I, the car will compensate more on the past error it makes.
High I will make the steer oscillates more to further reduces the sum of past errors.
Low I will oscillate less which reduces the accumulated error slower.

D, derivative control focuses on the rate of change. In our case, D controls the time for the car reacting to the angle.
High D will make the car less sensitive which contribute to overdamp where it is slow to approach the optimum track.
Low D will make the car swivels more even following the optimum track.


## Describe how the final hyperparameters were chosen.
I started with the Twiddle I have implemented in the PID class. The Twiddle gave me the parameters of 0.78209,0,6.1051 after 15 iterations.
The car was able to complete the track but I notice the car swivel too much when turning and coming out from the bridge.
I then adjust the parameter by reducing P to 0.18209 and the number of sharp steering angle reduces and the number of swivels when driving straight reduces.
I then further reduce the D to 0.61051 because I notice the car takes a long time to correct the error.
I did not adjust the I because mostly the road does not bend too much.
Finally, I settled down with P=0.18209 I=0 and D=0.61051. The car drives better.

After that, I implement another PID for speed control which maintain the speed at 30mph.

However, in the future I will prefer to tune the PID with Ziegler-Nichols method.
The method uses both critical gain and critical period to better tune the PID by following a heuristics table values.
