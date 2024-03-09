# RoboticARM_2024

## Task
Task 225 is to Create a [program](src/arm_control/src/start_arm.cpp) to take angle input for robotic arm.

## Status
Finished writing the program to input string of format 'a,b,c,d,e,f' where each is the angle to turn for each joint starting from the base to the claw. Input is given to ESP in Robotic Arm via the Due.

## Program
It takes input as a string of 6 angles separated by commas and is published as Topic 'Movement'.

### Change Log

| Date       | Task No. | Task Name           | Code Changes       | Added Publisher | Added Subscriber |
|------------|----------|---------------------|--------------------|-----------------|------------------|
| 29/02/2024 | 225      | Forward Kinematics  | Added a Ros package to take Input   | Movements               | -              |
