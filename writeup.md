## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[image1.5]: ./misc_images/misc1.5.png
[image4]: ./misc_images/misc4.png
[image5]: ./misc_images/kuka_pickplace.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is the annotated figure of the robot.

![alt text][image1]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193 | 0

In DH parameters,  
`alpha(i-1)` : Angle between Z(i-1) and Z(i) measured along X(i-1)   
`a(i-1)` : Link length, distance between Z(i-1) and Z(i), measured along X(i-1)   
`d(i)`  : Link offset, distance between X(i-1) and X(i), measured along Z(i-1), variable in prismatic joints(there are no prismatic joints in the given problem)   
`theta(i)` : Joint angle, Angle between X(i-1), X(i) measured along Z(i), variable in revolute joints.   
Also, 

In this robot arm,
`Link 0` represents the base link, and   
`Link 7` represents the gripper link, which is fixed.  

**Link 1 :** `Z0`(0 0 1) is *collinear* to `Z1`(0 0 1), `alpha0 = 0`, `a0 = 0`, `d1 = 0.33(joint1.z) + 0.42(joint2.z) = 0.75`, and `theta1` is *unknown*. `0.33` and `0.42` can be found in <joint> in the urdf file. 

**Link 2 :** `Z1`(0 0 1) is *perpendicular* to `Z2`(0 1 0), so, `alpha1 = -pi/2` (it is much easier to look at the picture above), `a1 = 0.35(joint2.x)`, and `d2 = 0` since `X1` intersects `X2` at `O2`. Also, we can see that when joint2 is in *zero* configuration, there is always an offset of `-pi/2` from `X1` to `X2`, measured along `Z2`. So, we also need to substitute `theta2` with `theta2 - pi/2` in the parameter table.   

**Link 3 :**, since `Z2`(0 1 0) is *parallel* to `Z3`(0 1 0), `alpha2 = 0`, `a2 = 1.25(joint3.z)` along `X2`. Also, `X2` and `X3` are collinear, so `d3 = 0`.   

**Link 4 :**, `Z3`(0 1 0) and `Z4`(1 0 0) are *perpendicular*, so `alpha3 = -pi/2` and `a3 = -0.054(joint4.z)`, and `d4 = 0.96(joint4.x) + 0.54(joint5.x) = 1.50`.   
*Note:* We have choosen O4, O5 and O6 to be co-incident with the Wrist Center(WC). This helps in separating the IK problem into computation of the Wrist Center and then Wrist Orientation.   

**Link 5 :**, `Z4`(1 0 0) and `Z5`(0 1 0) are *perpendicular* and *intersect* at `O5`, so `alpha4 = pi/2` and `a4 = 0`. Also, `d5 = 0`, since `X4` and `X4` are *collinear*.   

**Link 6 :**, `Z5`(0 1 0) and `Z6`(1, 0, 0) are *perpendicular* and *intersect* at `O5`, so `alpha5 = -pi/2` and `a5 = 0`, `d6 = 0`, since `X5` and `X6` are *collinear*.   

**Link 7(Gripper Link) :**, this is a fixed link, with a translation along `Z6`. So, `Z6` and `Zg` are *collinear*, so `alpha6 = 0`, `a6 = 0` and `d6 = 0.193(joint6.x) + 0.11(gripper_joint.x)`. Also, since this is fixed(w.r.t link 6), `q7 = 0`.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.  

This is the transformation matrix from i-1 to i
![alt text][image1.5]

An individual transformation matrix can be created like this,
```python
def createMatrix(alpha, a, q, d):
    mat =  Matrix([[            cos(q),           -sin(q),           0,             a],
                   [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                 0,                 0,           0,             1]])

    return mat
```   
with proper sets of parameters.

A generalized homogeneous transform between base_link and gripper_link can be achieved by multiplying these matrices together.
`T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G`  

To get two frames to align, `T0_G` needs to be rotated `pi` about the Z axis and `- pi` about Y axis.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

The last three angles (q4, q5, q6) do not affect the position of of wrist center (the intersection of z-axis of the last three joints). So we can divide IK problem into two problems, position (contolled by q1, q2, q3) and orientation (controlled by q4, q5, q6).

I spent a lot of time trying to solve the IK problem until I found this image from "https://github.com/jaycode/RoboND-Kinematics-Project/README.md"

![alt_text][image4]

At first, `theta1` can be solved by looking at the arm from top and use basic geometry.

By this image, `theta2` and `theta3` can be solved easily with geometry.

To seperate orientation from position. The formula below is used.

![alt text][image3]

After seperation, `theta4`, `theta5` and `theta6` can be solved by `atan2` function.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

The main part of my solution is described above. The Inverse Kinematics problem can be decoupled into Inverse Position Kinematics and Inverse Orientation Kinematics, which are easy to solve with geometry. It is very hard to calculate the angles until the digram is drawn.

The hardest problem I have encountered in this project is that the symbollic rotational matrix is overwritten in the loop. As a result, the rotational matrix will never be updated in the later loops because `.eval()` function will never find the symbols to replace after the first run.

The result is good enough. Every can goes to the bin eventually. One of the potential imporvement is that the angles can be normalized into the range between 0 to pi so that one joint would not rotate one whole circle but do nothing in fact.

![alt text][image5]
