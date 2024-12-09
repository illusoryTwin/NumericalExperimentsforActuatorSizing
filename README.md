# Numerical experiments 

**The second part of the actuator sizing task: choosing custom motors for the UR3 manipulator.**

The first part can be accessed in this [repository](https://github.com/illusoryTwin/ActuatorSizing/tree/main).


For the motor selection algorithm, we can define the follwoing main aspects that we need to consider:

- Geometrical sizes 
- Inertia Ratio, Gear Ratio
- Torques (Max allowed torques)

## Project Structure 

- In a `model` folder, you can find all 4 xml models with different motors  
- In the `src` folder, you can see the source code for the conducted experiments
- In a `data` folder, you can find the torque values for each of the 4 models
- In the `plots` folder, you can see the plots of torques as a result of the experiments
- Report about motor selection methodology, which was used in this task, can be found in `report.pdf`