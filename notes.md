# simulation steps

1. init states x(0)
2. compute forces and torques
3. invoke ode solver to compute x(t + dt) (i.e. 显式积分， x(t + dt) = x(t) + dx )
4. update state
5. t = t + dt, and then go to 2