# aed_mini_project
Advanced Electrical Drives SS2021 Mini-Project

The primary goal of the project was to develop and program an algorithm to compute the required reference currents (direct and quadrature axis) for controlling a permanent magnet synchronous machine. The algorithm is capable of handling both salient and non-salient machines.  The following trajectories are considered for control purposes.

## 1. MTPA
Maximum-torque-per-ampere: the machine will deliver the requested torque from 0 rpm until such a speed, where the voltage limits of the machine are reached, with the minimum amount of current. 

## 2. Lines of constant torque | Basic field weakening
For machine speeds where MTPA operation is not possible, the currents will be selected to reduce the machine's magnetic flux. This enables the delivery of the requested torque while maintaing voltage limits. This is employed up to speeds where the machine's current limit will be reached or the MTPF line is reached. 

## 3. MA
Maximum-ampere: At sufficently high speeds the machine will no longer be able to deliver the requested torque due to maximum stator current limits. At this point the maximum possible torque will be delivered with rated stator current.

## 4. MTPF
Maximum-torque-per-flux-linkage: At the highest speeds of operation, it becomes necessary to reduce the machine's magnetic flux even further to maintain voltage limits, resulting in a further reduction of torque output. Operation along the MTPF line in this speed region ensures maximum torque output for the given speed.  
