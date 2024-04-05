# 3D Data Processing
 3D Data Processing Master Degree Course Assignments
 
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
- Declares cv::Mat objects to store the images load from files.
- Loads the images making them grayscaled.
- Creates an instance of the **SGM** class with the specified disparity range.
- Uses the **set** method of the **SGM** class to set left and right images along with the monocular right image.
- Compute disparity map with **compute_disparity** method.
- Saves the disparity map.
- Calculates the Mean Squared Error between the computed disparity map and the ground truth disparity map.

#### sgm.h

This code defines the class SGM.
Class Declaration (**SGM**):
- **Public Member Functions**:
	- **set**: Sets input images and performs some preprocessing
	- **compute_disparity**: Computes disparity maps using SGM algorithm
	- **save_disparity**: Saves the computed disparity map to a file
	- **compute_mse**: Computes Mean Squared Error (MSE) between the computed disparity map and a ground truth disparity map
	
- **Private Member Functions**:
	- **init_paths**: Initializes paths for path calculation
	- **aggregation**: Aggregates costs over path
	- **calculate_cost_hamming**: Calculates cost based on Hamming distance
	
- **Private Member Variables**: Various parameters and data structures used in the algorithm, such as image dimensions, disparity range, penalty parameters (p1, p2), threshold for confidence, window dimensions, matrices for disparity and monochrome images, arrays for cost computation, etc.

