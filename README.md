# Numerical experiments 

**The second part of the actuator sizing task: choosing custom motors for the UR3 manipulator.**

The first part can be accessed in this [repository](https://github.com/illusoryTwin/ActuatorSizing/tree/main).


For the motor selection algorithm, we can define the follwoing main aspects that we need to consider:

- Geometrical sizes 
- Inertia Ratio, Gear Ratio
- Torques (Max allowed torques)

## Project Structure 

- In a `model` folder, you can find all 4 xml models with different motors  
- In a `data` folder, you can find the torque values for each of the 4 models

## Motor Selection Algorithm Steps:

While choosing the motors for the UR3 manipulator, I focused on the following criteria:

1. **Sizes** 

Firstly, I checked the sizes of the motors and compared them with the sizes of the manipulator.

\begin{figure}[h]
\centering
\includegraphics[width=\linewidth]{/report_src/manipulator_sizes.png}
\caption{Manipulator Sizes}
\end{figure}

\begin{figure}[h]
\centering
\includegraphics[width=\linewidth]{/report_src/motor_sizes.png}
\caption{Motor Sizes}
\end{figure}

These comparisons led me to the following options for motors:

- CRA-RI80-110-PRO-XX
- CRA-RI80-110-NH-XX

2. **Gear Ratios**

I chose the gear ratios with the following reasoning in mind:  
Shoulder joints require more powerful motors, which means higher gear ratios. In contrast, the wrist joint requires the 
lowest gear ratio since it needs to rotate with a relatively high speed.  

Therefore, for the shoulder joints, I used large gear ratios, 161:1 or 121:1.  
For the elbow joint, I used a smaller ratio of 121:1 or 101:1.  
The smallest gear ratio was used for the wrist joint, with a ratio of 51:1.

3. Based on criteria 1 and 2, I created four configuration combinations for the manipulat

You can find them in this \href{repo}{https://github.com/illusoryTwin/ActuatorSizing/tree/main}. 

4. I conducted both static and dynamoc experiments to calculate the torque values on motors in a static position and while movement accordingly. 

Next, I compared the obtained values with the maximum allowable torque from the motor's reference to ensure the motor does not exceed these limits.  

As a result, I found that the chosen motors for all the joints, except for the shoulder pan joint, satisfy the requirements. 
Next, I compared the obtained values with the maximum allowable torque from the motor's reference to ensure the motor does not exceed these limits.  

As a result, I found that the chosen motors for all the joints, except for the shoulder pan joint, satisfy the requirements.
















