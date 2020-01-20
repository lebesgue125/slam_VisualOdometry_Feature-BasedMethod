# SLAM  Visual Odometer : Feature-Based Method
The project implements the front end of the SLAM system, also known as the VO (Visual Odometer). The algorithm of VO are mainly divided into two categories: Feature-Based Method and Direct Method.

- **ORB Feature extraction and matching**

This project applies the feature based method and uses the ORB (Oriented FAST and Rotated BRIEF) as the feature extraction algorithm. The reason using ORB is that it is now a very representative real-time image feature. It improves the problem that FAST detector can not describe the direction, and adopts the binary descriptor BRIEF with extremely fast speed, so that the whole process of image feature extraction is greatly accelerated. 

BRIEF describes the vector composed of many 0 s and 1 s, which coding the relationships of two randomly chosen pixels (such as p and q) nearby given key point:  if p is greater than q, the value in this dimension of the vector is 1, and vice is 0. If we take 128 pairs of such pixels , you end up with 128 dimensions of 0,1 vectors.  

Use Brute-Force Matcher as a feature matching method.   For binary descriptors, such as BRIEF, use the Hamming distance as a metric -- the Hamming distance between two binary strings, referring to the number of different bits.

- **2D-2D: Epipolar Geometry & Epipolar Constraint**
  
According to the mathematical principle, handwritten realization from the essence matrix E, to get the rotation R and translation t.
1. Decompose E by SVD
   <div align=center>
   <img src="https://latex.codecogs.com/gif.latex?\dpi{130}&space;\large&space;E=U&space;\Sigma&space;V^T" />
   </div>  
2. Singular Value, assume <img src="https://latex.codecogs.com/gif.latex?\Sigma=diag(\sigma_1,\sigma_2,\sigma_3)" /> and <img src="https://latex.codecogs.com/png.latex?\dpi{100}&space;\sigma_1&space;\ge\sigma_2\ge\sigma_3" /> then.
   
   <div align=center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{130}&space;\large&space;\Sigma=diag(\frac{\sigma_1&plus;\sigma_2}{2},\frac{\sigma_1&plus;\sigma_2}{2},0)"/>
   </div>  

3. Four possible solutions
   <div align=center>
   <img src="https://latex.codecogs.com/gif.latex?\dpi{130}&space;\large&space;\begin{aligned}&space;&&space;t_1^{\land}=UR_Z(\frac{\pi}{2})\Sigma&space;U^T,&space;\qquad&space;R_1=UR_Z^T(\frac{\pi}{2})&space;V^T&space;\\&space;&&space;t_2^{\land}=UR_Z(-\frac{\pi}{2})\Sigma&space;U^T,&space;\qquad&space;R_2=UR_Z^T(-\frac{\pi}{2})&space;V^T&space;\\&space;\end{aligned}"/>
   </div>  

    Where <img src="https://latex.codecogs.com/png.latex?\inline&space;R_Z(\frac{\pi}{2})" /> represents the rotation matrix obtained by rotating 90 degrees along the Z axis. Using AngleAxis or Sophus::SO3 calculate <img src="https://latex.codecogs.com/png.latex?\inline&space;R_Z(\frac{\pi}{2})"/>.



- **3D-2D: using G-N to estimate the pose in Bundle Adjustment**
  
BA (Bundle Adjustment) is the least squares of  Reprojection error problem. BA can be implemented by Ceres and g2o and can be used for pose estimation in PnP. Write a gauss Newton method by hand to realize the function of using Bundle Adjustment to optimize the pose and figure out the camera pose.

Build the least square problem, then find the best camera pose to minimize it:


   <div align=center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{130}&space;\large&space;\xi^*=\mathop{argmin}\limits_\xi\frac{1}{2}\sum_{i=1}^n&space;\Big\|\mu_i-\frac{1}{s_i}K\exp(\xi^{\land})P_i\Big\|_2^2."/>
   </div>  


2X6 Jacobian Matrix:
   <div align=center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{130}&space;\large&space;J=\frac{\partial&space;e}{\partial&space;\delta&space;\xi}=-\begin{bmatrix}&space;\frac{f_x}{Z'}&space;&&space;0&space;&&space;-\frac{f_xX'}{Z'^2}&-\frac{f_xX'Y'}{Z'^2}&f_x&plus;\frac{f_xX^2}{Z'^2}&&space;-\frac{f_xY'}{Z'}\\&space;0&space;&&space;\frac{f_y}{Z'}&&space;-\frac{f_yY'}{Z'^2}&-f_y-\frac{f_yY'^2}{Z'^2}&\frac{f_xX'Y'}{Z'^2}&&space;\frac{f_yX'}{Z'}&space;\end{bmatrix}."/>
   </div>  

On the other hand, in addition to optimizing posture, we also want to optimize the spatial position of feature points.
   <div align=center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{130}&space;\large&space;J=\frac{\partial&space;e}{\partial&space;P}=-\begin{bmatrix}&space;\frac{f_x}{Z'}&space;&&space;0&space;&&space;-\frac{f_xX'}{Z'^2}\\&space;0&space;&&space;\frac{f_y}{Z'}&&space;-\frac{f_yY'}{Z'^2}&space;\end{bmatrix}R."/>
   </div>  


- **3D-3D: Trajectories Alignment by ICP**

In practice, we often need to compare the error between two trajectories.

Let the true trajectory be <img src="https://latex.codecogs.com/png.latex?T_g">, and the estimated trajectory be <img src="https://latex.codecogs.com/png.latex?T_e">; The theoretical real trajectory point and estimated trajectory point should satisfy:

   <div align=center>
   <img src="https://latex.codecogs.com/png.latex?\dpi{130}&space;\large&space;T_{g,i}=T_{ge}T_{e,i}"/>
   </div>  

Where <img src="https://latex.codecogs.com/png.latex?i"> represents the <img src="https://latex.codecogs.com/png.latex?i">th record in the trajectory, <img src="https://latex.codecogs.com/png.latex?\inline&space;\dpi{100}&space;T_{ge}&space;\in&space;SE(3)"> is the transformation matrix between two coordinate systems, which remains unchanged in the whole trajectory.

The translation part of the two trajectories is regarded as the point set. Then using Iterative Closest Point (Iterative Closest Point, ICP) to solve the transformation between the Point set.

