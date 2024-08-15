# simulation steps

1. init states x(0)
2. compute forces and torques
3. invoke ode solver to compute x(t + dt) (i.e. 显式积分， x(t + dt) = x(t) + dx )
4. update state
5. t = t + dt, and then go to 2

# 长方体判交

先不使用基于witness的方法，以简化流程。

1. 判断bounding box是否相交
2. 6个面逐面判断是否相交 => 化为单纯形来判断，归结为三角形判交的问题
3. 判断三角形所在平面是否相交，交线是否同时穿过两个三角形