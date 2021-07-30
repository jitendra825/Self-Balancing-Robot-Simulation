# Self-Balancing-Robot-Simulation
![SBR_8-Shape](https://user-images.githubusercontent.com/88198964/127717390-5d413aef-fe3b-4031-b74a-a5e53549d86b.gif)

This repository contains simulation for Self Balancing Robot. LQR Control for Balancing the robot in vertical position. Feedback Linearization control for Trajectory Tracking
Steps for Run the Simulation
1. firstly, Run lqr_control_3DOF.m file in MATLAB to calculate gains & initilize the parameters of system
2. To simulate Feedback linearization control for trajectory tracking of Self balancing Control open FDL_Trajectary_Tracking_SBR.slx in Simulink

![tempsnip](https://user-images.githubusercontent.com/88198964/127717279-8748becb-79af-4e5f-a763-a157682965a1.png)

3. To change the Trajectory types Click on block showing in above Image.
4. Type Number between 1 to 4 such as

   1 for Circular Trajecctory
   
   2 for 8-Shaped Trajecotory
   
   3 for Infinite Shaped Trajectory
   
   4 for Heart Shaped Trajectory (Just for Fun/ this is not a Continious trajectory)
   
5. Run the simulation by click on RUN Icon in Simulink
6. To Visualizing the 3D Animation,Double click on VR Sink block and run the simulation in VRML
