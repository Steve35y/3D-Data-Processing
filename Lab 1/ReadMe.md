# Lab 1 - Semi Global Stereo Matching with Monocular Disparity Initial Guess

Semi-global matching (SGM) is a computer vision algorithm to estimate the depth or disparity map of a scene from a pair of stereo images. It's a refinement of the traditional stereo matching approach that aims to improve accuracy and robustness, particularly in challenging scenarios such as occlusions, textureless regions, and lighting variations.

- Binocular disparity refers to the difference in image location of an object seen by the left and right eyes, resulting from the eyes’ horizontal separation (parallax). 

- Stereo imaging is a technique for creating or enhancing the illusion of depth in an image by means of stereopsis for binocular vision. Here we rectify the image pair into a common image plane being one of the two, left or right, image planes.

## In short, its functioning:
1. Stereo Images Acquisition: Capturing a pair of stereo images of the same scene from two slightly different viewpoints. 
2. Feature Extraction: Corresponding features (edges, corners, texture patterns...) are identified, serving as reference points for matching.
3. Cost Calculation: For each pixel in one image the goal is to find the corresponding pixel in the other image. To do this, a cost function is computed for each possible disparity value between the left and right pixels. The cost function measures the similarity between the pixels at the current position and their potential matches at different disparities. 
4. Aggregation of Costs: In semi-global stereo matching, instead of only considering the local cost at each pixel independently, the algorithm aggregates costs over a local region or along certain paths. This helps to incorporate global information into the matching process, which improves accuracy and reduces errors. The aggregation can be performed along multiple directions (h,v,d) and across multiple scales.
5. Disparity Computation: After aggregating costs, the algorithm computes the disparity map by selecting the disparity value that minimizes the aggregated cost for each pixel. This disparity map represents the estimated depth or the shift in pixel position between corresponding points in the stereo images.
6. Post-processing: Optional, for refining.
7. Evaluation and Optimization: The performance of the semi-global stereo matching algorithm is evaluated based on metrics such as accuracy, 


## In the assignment:
An incomplete source code of a basic SGM algorithm is provided.
The main.cpp source file defines the main function that calls the SGM() class and computes the quantitative results (i.e., the MSE errors) on the sample data item provided as input argument.

It already implements:
- The right-to-left cost volume computation.
- Part of the cost aggregation calculation.
- Right-to-left disparity and confidence final computation.
- Quantitative assessment, by means of the Mean Squared Error (MSE) computation
of the right-to-left disparity map.

## My implementation

1. *compute_path_cost() function*:

Given a single pixel *p*, defined by its coordinates cur_x and cur_y, a path with index *cur_path*, direction increments along the path *direction_x*, *direction_y*, should compute the path cost for *p* for all the possible disparities d from 0 to disparity_range_. The output should be stored in the tensor (already allocated) *path_cost_[cur_path][cur_y][cur_x][d]*, for all possible d. That is, at each call of *compute_path_cost()*, given a pixel with coordinates.

In particular the path_cost formula is the following:
$$L_r(p,d) = C(p,d) + min(L_r(p-r,d), L_r(p-r,d-1) + P1, L_r(p-r,d+1) + P1, min_i(L_r(p-r,i)) + P2) - min_k(L_r(p-r,k))$$































	
