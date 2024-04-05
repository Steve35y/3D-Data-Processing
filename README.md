# 3D Data Processing
 3D Data Processing Master Degree Course Assignmetns
 
## Lab 1 - Semi Global Stereo Matching with Monocular Disparity Initial Guess

Semi-global matching (SGM) is a computer vision algorithm for the estimation of a dense disparity map from a rectified stereo image pair.

- Binocular disparity refers to the difference in image location of an object seen by the left and right eyes, resulting from the eyes’ horizontal separation (parallax). 

- Stereo imaging is a technique for creating or enhancing the illusion of depth in an image by means of stereopsis for binocular vision. Here we rectify the image pair into a common image plane being one of the two, left or right, image planes.

### In the assignment:
An initial guess of the disparity map is provided calculated using a very recent data-driven monocular depth estimation method, generally very accurate, defined up to a scalar factor.

The scalar factor has to be computed once the disparity map with SGM is, and used the scaled initial guess disparity map to refine the disparity map computed with SGM.

#### main.cpp

- Includes the header file sgm.h for the SGM algorithm implementation.
- Defines the main function taking command-line arguments 'argc' and 'argv'.
- It has to take 7 arguments, in particular
	- [1] File path of the right image.
	- [2] File path of the left image.
	- [3] File path of the monocular right image.
	- [4] File path of the ground truth disparity map.
	- [5] File path for the output disparity image.
	- [6] Disparity range.
	 
	
