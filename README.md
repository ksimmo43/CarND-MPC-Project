# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model Description

The MPC (Model Predictive Control) model consists of a Kinematic vehicle model that ignores dynamic forces (such as tire forces) in order to predict future states of the vehicle. An optimization technqiue then uses this model and any given bounds and constraints to minimize a cost function over a prediction horizen. The actuator values for the first time step of the optimization solution are then used as the next control setpoint. The rest of the solution is thrown away and the process is repeated. The following is the vehicle model that was used to make predicitons:

x<sub>​t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos(ψ<sub>t</sub>) Δt

y<sub>​t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin(ψ<sub>t</sub>) Δt

ψ<sub>​t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> tan(δ<sub>t</sub>) Δt / L<sub>f</sub>

v<sub>​t+1</sub> = v<sub>t</sub> + α<sub>t</sub> Δt

state = {x, y, ψ, v} (x,y position, vehicle heading angle, velocity)

actuators = {δ, α} (Front Road wheel angle, throttle)

The cross track error and heading angle error are used to measure how well the vehicle is aligned with the reference trajectory. These are calculated using the following equations and are added to the vehicle state matrix.

cte = cross-track error

eψ = ψ angle error

cte<sub>​t+1</sub> = cte<sub>t</sub> + v<sub>t</sub> sin(eψ<sub>t</sub>) Δt

eψ<sub>​t+1</sub> = eψ<sub>t</sub> + v<sub>t</sub> tan(δ<sub>t</sub>) Δt / L<sub>f</sub>

state = {x, y, ψ, v, cte, eψ}

actuators = {δ, α}

These error terms, as well as other terms used to minimize actuator usage and attempts to smooth the control output, are combined to form the cost function that the optimizer minimizes. Weighting values are added to each term to place more importance on certain terms. The cost function is defined in lines 34-55 of the mpc.cpp file. 

## Hyperparameters
MPC performs N predictions using Δt between each prediction steps. These parameters are chosen considering a trad-off between prediciton resolution and length, and the computation time needed to complete the prediciton and optimization. Large N and small Δt give great model resolution, but could be too computationally expensive. For me, making N too large led to errors at high speed because predicting very far in the future was not useful due to limited knowledge of the reference trajectory. Making Δt too small led to very jittery control. In the end, I settled on N = 10 and Δt = 0.1
The MPC optimization algorithm performs N predictions of the state and actuator vectors after Δt has elapsed. N and Δt are two very important parameters to tune the algorithm.

I also had to tune the cost function weights to get good performance. I placed large importance on minimzing CTE and also on having smooth steering control. The weights I chose can be seen in mpc.cpp

## Polynomial Fit
First, the reference trajectory points received from the simnulator were translated into the vehicle's coordinate frame. Then a polynomial is fit to the points. I chose a 4th order polynomial because around sharp turns, lower order polynomials caused errors in the optimization solution at some steps.

## Latency
Latency is accounted for by modifying the initial vehicle state that is given to the optimizer. We predict the vehicle location based on the current actuator positions by the anticipated latency amount. This would be the actual time that the next control action would be taken by the MPC solution. In this way, we can accurately account for the latency.
