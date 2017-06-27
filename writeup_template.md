## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---



[//]: # "Image References"

[image1]: ./misc_images/arm_calc.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

This is the table containing the Modified DH parameters.


| $\mathbf i$ | $\mathbf \alpha_{i-1}$ | $\mathbf a_{i-1}$ | $\mathbf d_i$ | $\mathbf \theta_{i}$                 |
| ----------- | ---------------------- | ----------------- | ------------- | ------------------------------------ |
| 1           | 0                      | 0                 | 0.75          |                                      |
| 2           | $-\frac{\pi}{2}$       | 0.35              | 0             | $\theta_2 = \theta_2 -\frac{\pi}{2}$ |
| 3           | 0                      | 1.25              | 0             |                                      |
| 4           | $-\frac{\pi}{2}$       | -0.054            | 1.50          |                                      |
| 5           | $\frac{\pi}{2}$        | 0                 | 0             |                                      |
| 6           | $-\frac{\pi}{2}$       | 0                 | 0             |                                      |
| 7           | 0                      | 0                 | 0.303         | $\theta_7 = 0$                       |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

$$
T^0_1 = \begin{bmatrix}
			cos(\theta_1)& -sin(\theta_1)& 0& 0 \\
			sin(\theta_1)& cos(\theta_1)& 0& 0 \\
			0& 0& 1& 0.75 \\
			0& 0& 0& 1
		\end{bmatrix}

T^1_2 = \begin{bmatrix}
			sin(\theta_2)& cos(\theta_2)& 0& 0.35 \\
			0& 0& 1& 0 \\
			cos(\theta_2)& -sin(\theta_2)& 0& 0 \\
			0& 0& 0& 1
		\end{bmatrix}
		
\notag
$$

$$
T^2_3 = \begin{bmatrix}
			cos(\theta_3)& -sin(\theta_3)& 0& 1.25 \\
			sin(\theta_3)& cos(\theta_3)& 0& 0 \\
			0& 0& 1& 0 \\
			0& 0& 0& 1
		\end{bmatrix}

T^3_4 = \begin{bmatrix}
			cos(\theta_4)& -sin(\theta_4)& 0& -0.054 \\
			0& 0& 1& 1.50 \\
			-sin(\theta_4)& -cos(\theta_4)& 0& 0 \\
			0& 0& 0& 1
		\end{bmatrix}
\notag
$$

$$
T^4_5 = \begin{bmatrix}
			cos(\theta_5)& -sin(\theta_5)& 0& 0 \\
			0& 0& -1& 0 \\
			sin(\theta_5)& cos(\theta_5)& 0& 0 \\
			0& 0& 0& 1
		\end{bmatrix}

T^5_6 = \begin{bmatrix}
			cos(\theta_6)& -sin(\theta_6)& 0& 0 \\
			0& 0& 1& 0 \\
			-sin(\theta_6)& -cos(\theta_6)& 0& 0 \\
			0& 0& 0& 1
		\end{bmatrix}
\notag
$$

$$
T^6_G = \begin{bmatrix}
			1& 0& 0& 0 \\
			0& 1& 0& 0 \\
			0& 0& 1& 0.303 \\
			0& 0& 0& 1
		\end{bmatrix}
$$


Given the target orientation $roll$, $pitch$ and $yaw$ we have the following rotation matrices:
$$
R_{roll} = \begin{bmatrix}
1 & 0 & 0 \\
0 & cos(roll) & -sin(roll) \\
0 & sin(roll) & cos(roll)
\end{bmatrix}
\\
R_{pitch} = \begin{bmatrix}
cos(pitch) & 0 & sin(pitch) \\
0 & 1 & 0 \\
-sin(pitch)& 0 & cos(pitch)
\end{bmatrix}
\\
R_{yaw} = \begin{bmatrix}
cos(yaw) & -sin(yaw)& 0 \\
sin(yaw) & cos(yaw) & 0 \\
0 & 0 & 1
\end{bmatrix}
\\
\notag
$$
And thus:
$$
R^0_6 = R_{yaw} \dot{} R_{pitch} \dot{} R_{roll} = 
\begin{bmatrix}
			l_x & m_x & n_x \\
			l_y & m_y & n_y\\
			l_z & m_z & n_z
\end{bmatrix}
$$
Given also the target position $p_x$, $p_y$ and $p_z$ , the complete target matrix can now be defined as follows:

$$
T^0_G = \left[
\begin{array}{c|c}
R^0_6 & \begin{matrix}p_x \\ p_y \\ p_z \end{matrix} \\
\hline
0 & 1
\end{array}
\right] = 
\begin{bmatrix}
			l_x & m_x & n_x & p_x \\
			l_y & m_y & n_y & p_y \\
			l_z & m_z & n_z & p_z \\
			0& 0& 0& 1
		\end{bmatrix}
$$

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![alt text][image1]

First we need to calculate the Wrist Center vector $WC$.
$$
WC = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix} - d_7 \dot{} R^0_6 \dot{}\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
\implies 
\begin{cases} 
WC_x = p_x - (d_7 \dot{} n_x) \\
WC_y = p_y - (d_7 \dot{} n_y)\\
WC_z = p_z - (d_7 \dot{} n_z)
\end{cases}
$$
Now that we have the $WC$, calculating $\theta_1$ is fairly trivial.
$$
\theta_1 = atan2(WC_y,WC_x)
$$

Before calculating $\theta_2$ and $\theta_3$ we need to calculate the length of the sides of the triangle $[a_2,l,g]$:
$$
\\
l = \sqrt{a^2_3 + d^2_4} \\
\\
g = WC - t^0_2 = WC - \begin{bmatrix} a_1 \dot{} cos(\theta_1) \\ a_1 \dot{} sin(\theta_1) \\ d_1 \end{bmatrix} =  \begin{bmatrix} g_x \\ g_y \\ g_z \end{bmatrix} \\
\\
|g|=\sqrt{g_x^2 + g_y^2 +g_z^2 }
$$
Using the cosine rule...
$$
A^2 = B^2 + C^2 - 2BCcos(\alpha) \\
\alpha = arccos \left(\frac{B^2 + C^2 - A^2}{2BC} \right)\\
\notag
$$
... and the conversion into $atan2$ as follows ...
$$
arccos(x) = atan2(\sqrt{1-x^2},x)\\
\notag
$$
We can now start finding $\theta_3$:
$$
\varphi = atan2(d_4, a_3) \\
D = \left(\frac{ l^2 + a^2_2 - |g|^2}{2la_2}\right) \\
\delta = atan2(\sqrt{1-D^2},D) \\
\theta_3 = \varphi - \delta
$$
And also $\theta_2$:
$$
g_{xy} = \sqrt{g_x^2 + g_y^2}\\
\alpha = atan2(g_z,g_{xy}) \\
D = \left(\frac{|g|^2 + a^2_2  - l^2}{2|g|a_2}\right) \\
\beta = atan2(\sqrt{1-D^2},D) \\
\theta_2 = \frac{\pi}{2} - \alpha - \beta
$$

Now we can proceed to the Inverse Orientation Kinematics.

$$
R^3_6 = (R^0_3)^T \dot{} R^0_6 
= \begin{bmatrix} r_{11} & r_{12} & r_{13} \\ r_{21} & r_{22} & r_{23} \\ r_{31} & r_{32} & r_{33} \end{bmatrix}
$$
$$
\theta_4 = atan2(r_{32}, r_{33})\\
\theta_5 = atan2(-r_{31}, \sqrt{r_{11}^2 + r_{21}^2})\\
\theta_6 = atan2(r_{21},r_{11})
$$

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  





