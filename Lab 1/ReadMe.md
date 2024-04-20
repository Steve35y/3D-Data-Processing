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

# My implementation

## compute_path_cost()

Given a single pixel **p**, defined by its coordinates **cur_x** and **cur_y**, a path with index **cur_path**, direction increments along the path **direction_x**, **direction_y**, should compute the path cost for **p** for all the possible disparities d from 0 to disparity_range_. The output should be stored in the tensor (already allocated) **path_cost_[cur_path][cur_y][cur_x][d]**, for all possible d. That is, at each call of **compute_path_cost()**, given a pixel with coordinates.

In particular the path_cost formula is the following:
$$L_r(p,d) = C(p,d) + min(L_r(p-r,d), L_r(p-r,d-1) + P1, L_r(p-r,d+1) + P1, min_i(L_r(p-r,i)) + P2) - min_k(L_r(p-r,k))$$


```c++
  void SGM::compute_path_cost(int direction_y, int direction_x, int cur_y, int cur_x, int cur_path)
  {
    unsigned long prev_cost, best_prev_cost, no_penalty_cost, penalty_cost, 
                  small_penalty_cost, big_penalty_cost;
 // if the processed pixel is the first:
    if(cur_y == pw_.north || cur_y == pw_.south || cur_x == pw_.east || cur_x == pw_.west)
    {
      // For pixels in the border the output L_r(p,d) is only equal to the Data term C(p,d).
      for (int d = 0; d < disparity_range_; ++d)
      {
      		// L_r(p,d) = C(p,d)
      		path_cost_[cur_path][cur_y][cur_x][d] = cost_[cur_y][cur_x][d];
      }
      
    }
```
So here for pixels in the borders, since the neightborhood isn't very reliable we consider the total cost equal as the Data term equal to the already calculated cost: $$L_r(p,d) = C(p,d)$$

```c++
    else
    {
      // Check the pixel is in bound
      if (cur_y - direction_y >= 0 && cur_x - direction_x >= 0 && cur_y - direction_y < height_ && cur_x - direction_x < width_) 
      {
	      for (int d = 0; d < disparity_range_; ++d)
	      {
	      		// L_r(p,d) = C(p,d) + min(L_r(p-r,d), L_r(p-r,d-1) + P1, L_r(p-r,d+1) + P1, min_i(L_r(p-r,i)) + P2) - min_k(L_r(p-r,k))
	      			
	      		// - Data term as before
	      		// C(p,d)
	      		no_penalty_cost = cost_[cur_y][cur_x][d];
```
First part of the Smoothness term is the previous pixel, along path direction, of the current pixel: $$prevcost = L_r(p-r,d)$$
```c++
	      		// - Smoothenss term: previous path cost
	      		prev_cost = path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][d];
```
Second part of the Smoothness term is the penalty cost due to small disparity changes, equal to: $$L_r(p-r,d-1) + P1 \text{ or } L_r(p-r,d+1) + P1$$
```c++
	      		// - Smoothness term: penalty term for small disparity changes 
	      		if ( d > 0 && d < disparity_range_ - 1)
	      		{
	      			small_penalty_cost = min(path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][d - 1] + p1_, path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][d + 1] + p1_);
	      		}
	      		else if ( d == 0)
	      		{
	      			small_penalty_cost = path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][d + 1] + p1_;
	      		}
	      		else if ( d == disparity_range_ - 1 )
	      		{
	      			small_penalty_cost = path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][d - 1] + p1_;
	      		}
```
Last part of the Smoothness term is the penalty cost due to big disparity changes, equal to: $$min_i(L_r(p-r,i)) + P2$$
```c++
	      		
	      		// - Smoothness term: penalty term for big disparity changes
	      		big_penalty_cost = ULONG_MAX; // Initialization
	      		for ( int i = 0; i < disparity_range_; ++i)
	      		{
	      			big_penalty_cost = std::min(path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][i], big_penalty_cost);
	      		}
	      		
	      		big_penalty_cost += p2_;
```
The Smoothenss term is, at the end, the minimum of the three calculated costs.
```c++	      		
	      		// Smoothness term as the minimum of the cost of the previous pixel, the small penalty cost, the big penalty cost
	      		// min(L_r(p-r,d), L_r(p-r,d-1) + P1, L_r(p-r,d+1) + P1, min(L_r(p-r,i)) + P2)
	      		penalty_cost = min(min(prev_cost, small_penalty_cost), big_penalty_cost);
```
TO DO $$min_k(L_r(p-r,k))$$
```c++	      		
	      		// - Restrict the range of resulting value, without affecting the minimization procedure
	      		// min_k(L_r(p-r,k))
	      		best_prev_cost = ULONG_MAX; // Initialization
	      		for ( int k = 0; k < disparity_range_; ++k)
	      		{
	      			best_prev_cost = min(best_prev_cost, path_cost_[cur_path][cur_y - direction_y][cur_x - direction_x][k]);
	      		}
```
The final output is computed as the above formula:
```c++	      		
	      		// output
	      		path_cost_[cur_path][cur_y][cur_x][d] = no_penalty_cost + penalty_cost - best_prev_cost;
	      }
	      
	    }
	}
  }
```
## aggregation()
In the **aggregation()** function I just had to initialize the  the variables **start_x**, **start_y**, **end_x**, **end_y**, **step_x** and **step_y** are used to define the cycles where **compute_path_cost()** is called.

```c++
      // Variables initialization
      start_x = (dir_x == 1) ? 0 : (dir_x == -1) ? width_ - 1 :  0;
      end_x = (dir_x == 1) ? width_ -1 : (dir_x == -1) ? 0  : width_ - 1;
      step_x = (dir_x == 1 || dir_x == -1) ? dir_x : 1 ;
      
      start_y = (dir_y == 1) ? 0 : (dir_y == -1) ? height_ - 1 : 0;
      end_y = (dir_y == 1) ? height_ -1 : (dir_y == -1) ? 0  : height_ - 1;
      step_y = (dir_y == 1 || dir_y == -1) ? dir_y : 1 ;
      
      for(int y = start_y; y != end_y ; y+=step_y)
      {
        for(int x = start_x; x != end_x ; x+=step_x)
        {
          compute_path_cost(dir_y, dir_x, y, x, cur_path);
          
        }
      }
    }
```
# Results without refinements
After the initial cost computation, for the 4 provided sample images: Aloe, Cones, Plastic, Rocks, I've calculated the disparity images and their MSE errors, accessible at [results](https://github.com/Steve35y/3D-Data-Processing/tree/main/Lab%201/Results).
For example the Aloe case:
```
MSE of the right image:  122.464
```
And the image result is:

![plot](../Lab%201/Results/Without%20refinements/output_disparity_Aloe.png)

## compute_disparity()
In the **compute_disparity()** function, use high confidence disparities to compute the linear coefficients that should be applied to the monocular disparity values in order to scale them.

Namely, given a pair of disparities of the same pixel $d_{sgm}$ and $d_{mono}$ representing the disparity estimated with SGM and the (unscaled) initial guess disparity, find $h$ and $k$ such that: $$d_{sgm} = h * d_{mono} + k$$

Recall that the solution for the least squares problem for a nonhomogeneous system $Ax = b$ is expressed as: $$x = (A^T A)^{-1} A^T b$$
where $x = [h k]^T$, $b = d_{sgm}$, $A = [d_{mono}, \overline{1}]$.

First create the struct to store the disparity vectors.
```c++
    struct DisparityPair 
  {
  	int d_mono;
  	int d_sgm;
  };

  void SGM::compute_disparity()
  {
      calculate_cost_hamming();
      aggregation();
      disp_ = Mat(Size(width_, height_), CV_8UC1, Scalar::all(0));
      int n_valid = 0;
      std::vector<DisparityPair> disparity_pair;
      for (int row = 0; row < height_; ++row)
      {
          for (int col = 0; col < width_; ++col)
          {
              unsigned long smallest_cost = aggr_cost_[row][col][0];
              int smallest_disparity = 0;
              for(int d=disparity_range_-1; d>=0; --d)
              {

                  if(aggr_cost_[row][col][d]<smallest_cost)
                  {
                      smallest_cost = aggr_cost_[row][col][d];
                      smallest_disparity = d; 

                  }
              }
              inv_confidence_[row][col] = smallest_cost - inv_confidence_[row][col];
```
After calculating the confindencies, find the disparities with good confidence and store them paired with the corrispondent monocular value:
```c++
              // If the following condition is true, the disparity at position (row, col) has a good confidence
              if (inv_confidence_[row][col] > 0 && inv_confidence_[row][col] <conf_thresh_)
              {

                uchar d_mono = mono_.at<uchar>(row, col);
                
                uchar d_sgm = smallest_disparity*255.0/disparity_range_;
                
                disparity_pair.push_back({d_mono, d_sgm});
                
                
                /////////////////////////////////////////////////////////////////////////////////////////
              }

              disp_.at<uchar>(row, col) = smallest_disparity*255.0/disparity_range_;

          }
      }
```
Now solve the least square problem:
```c++
      
       Eigen::MatrixXd A(disparity_pair.size(), 2);
       Eigen::VectorXd b(disparity_pair.size(),1);
      Eigen::VectorXd x;
      
      for (int i = 0; i < disparity_pair.size(); ++i) 
       {
       	A(i, 0) = disparity_pair[i].d_mono;
       	A(i, 1) = 1;
       	b(i) = disparity_pair[i].d_sgm;
       } 
      
      {
      		x = (A.transpose() * A).inverse() * A.transpose() * b;
      }
      float h = x(0);
      float k = x(1);
```
Improve the low confidence values with the scaled ones:
```c++      
      for (int row = 0; row < height_; ++row)
      {
      		for (int col = 0; col < width_; ++col)
      		{
      			if (!(inv_confidence_[row][col] > 0 && inv_confidence_[row][col] <conf_thresh_)){
      			disp_.at<uchar>(row, col) = h * mono_.at<uchar>(row, col) + k;
      			}	    			
      		}
      	}
  }
```
# Results after the refinement step
After running the last **compute_disparity()** function, the improvements of the results, about both images quality and MSE errors are considerable.
```
MSE of the right image:  13.7291
```
And the image result is:

![plot](../Lab%201/Results/With%20refinements/output_disparity_Aloe.png)


























	
