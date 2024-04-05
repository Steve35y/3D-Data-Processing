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
An initial guess of the disparity map is provided calculated using a very recent data-driven monocular depth estimation method, generally very accurate, defined up to a scalar factor.

The scalar factor has to be computed once the disparity map with SGM is, and used the scaled initial guess disparity map to refine the disparity map computed with SGM.

### main.cpp

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

### sgm.h

This code defines the class SGM.
Class Declaration (**SGM**):
- **Constructor**: **SGM** initializes various parameters used in the Semi-Global Matching algorithm.
- **Public Member Functions**:
	- **set**: Sets input images and performs some preprocessing
	- **compute_disparity**: Computes disparity maps using SGM algorithm
	- **save_disparity**: Saves the computed disparity map to a file
	- **compute_mse**: Computes Mean Squared Error (MSE) between the computed disparity map and a ground truth disparity map
	
- **Private Member Functions**:
	- **init_paths**: Initializes paths for path calculation
	- **aggregation**: Aggregates costs over path
	- **calculate_cost_hamming**: Calculates cost based on Hamming distance
	- **compute_path_cost**: Compute path cost
	
- **Private Member Variables**: Various parameters and data structures used in the algorithm, such as image dimensions, disparity range, penalty parameters (p1, p2), threshold for confidence, window dimensions, matrices for disparity and monochrome images, arrays for cost computation, etc.

## sgm.cpp / actual code implementation

Here, my code implementation is provided.

1. Definitions: **NUM_DIRS** and **PATHS_PER_SCAN** are defined as constants, representing the number of directions and paths per scan, respectively.

```C++
#define NUM_DIRS 3
#define PATHS_PER_SCAN 8
```

2. Global Variables:
	- **hamLut**: A lookup table for storing precomputed Hamming distances between all pairs of byte values (0-255).
	- **directions**: An array of integers representing directions for path initialization.

```C++
static char hamLut[256][256];
static int directions[NUM_DIRS] = {0, -1, 1};
```

3. Function Definitions:
- **compute_hamming_lut()**: Computes Hamming distances for all pairs of byte values and fills the hamLut table.
	
```C++
void compute_hamming_lut()
{
  for (uchar i = 0; i < 255; i++)
  {
    for (uchar j = 0; j < 255; j++)
    {
      uchar census_xor = i^j;
      uchar dist=0;
      while(census_xor)
      {
        ++dist;
        census_xor &= census_xor-1;
      }
      
      hamLut[i][j] = dist;
    }
  }
}
```

- **SGM constructor**: Initializes an instance of the SGM class with parameters like disparity range, penalties, confidence threshold, and window size.
	
```C++
// Declaration of the constructor
SGM::SGM(unsigned int disparity_range, unsigned int p1, unsigned int p2, float conf_thresh, unsigned int window_height, unsigned window_width):
//The part after the colon (':') is the initializer list. It's a comma-separated list of member variable initializations. Each item in the initializer list consists of the member variable name followed by (value), where value is the value to initialize the member variable with.
  disparity_range_(disparity_range), p1_(p1), p2_(p2), conf_thresh_(conf_thresh), window_height_(window_height), window_width_(window_width)
  {
// This is the body of the constructor. It contains the code that will be executed when an object of the SGM class is created. In this case, it calls the compute_hamming_lut() function to compute the Hamming distance lookup table.
    compute_hamming_lut();
  }
```

- **set()**: Sets the left and right images along with their dimensions.		
```C++
void SGM::set(const  cv::Mat &left_img, const  cv::Mat &right_img, const  cv::Mat &right_mono)
  {
  
    // Assign the input left and right images to the first two elements of the views_ array 
    views_[0] = left_img;
    views_[1] = right_img;
    
    // Assign the monocular image
    mono_ = right_mono;

    // Determine height and width of the left and right images
    height_ = left_img.rows;
    width_ = right_img.cols;
    
    // Calculate and assign values of the processing window
    pw_.north = window_height_/2;
    pw_.south = height_ - window_height_/2;
    pw_.west = window_width_/2;
    pw_.east = width_ - window_height_/2;
    
    // See next part
    init_paths();
    
    //Resize data structures for cost computation, aggregation, path costs. Parameters determined based on the dimension of the input images and other parameters.
    cost_.resize(height_, ul_array2D(width_, ul_array(disparity_range_)));
    inv_confidence_.resize(height_, vector<float>(width_));
    aggr_cost_.resize(height_, ul_array2D(width_, ul_array(disparity_range_)));
    path_cost_.resize(PATHS_PER_SCAN, ul_array3D(height_, ul_array2D(width_, ul_array(disparity_range_)))
    );
  }
```

- **init_paths()**: Initializes the path directions skipping the degenerate path.	

```C++
void SGM::init_paths()
  {
    // These loops iterate over each combination of directions defined by the directions array. 
    for(int i = 0; i < NUM_DIRS; ++i)
    {
      for(int j = 0; j < NUM_DIRS; ++j)
      {
        // skip degenerate path
        if (i==0 && j==0)
          continue;	// skip
          
        // This line creates a new path and adds it to paths_
        paths_.push_back({directions[i], directions[j]});
      }
    }
  }
```

- **calculate_cost_hamming()**: Computes the cost using the Hamming distance between census transformed pixel values of the left and right images.

```C++
void SGM::calculate_cost_hamming()
  {
  
    // Variables and matrices to store census transform results
    uchar census_left, census_right, shift_count;
    cv::Mat_<uchar> census_img[2];
    cv::Mat_<uchar> census_mono[2];
    
    cout << "\nApplying Census Transform" <<endl;
    
    // Loop over left and right images
    for( int view = 0; view < 2; view++)
    {
      census_img[view] = cv::Mat_<uchar>::zeros(height_,width_);
      census_mono[view] = cv::Mat_<uchar>::zeros(height_,width_);


      //// Loop over image rows
      for (int r = 1; r < height_ - 1; r++)
      {
      
        // Pointers to current rows in the images
        uchar *p_center = views_[view].ptr<uchar>(r),
              *p_census = census_img[view].ptr<uchar>(r);
        p_center += 1;
        p_census += 1;

        // Loop over image columns
        for(int c = 1; c < width_ - 1; c++, p_center++, p_census++)
        {
          uchar p_census_val = 0, m_census_val = 0, shift_count = 0;
          
          // Loop over neighborhood of the current pixel
          for (int wr = r - 1; wr <= r + 1; wr++)
          {
            for (int wc = c - 1; wc <= c + 1; wc++)
            {

              if( shift_count != 4 )//skip the center pixel
              {
                p_census_val <<= 1;
                m_census_val <<= 1;
                
                // Compare pixel values in the neighborhood
                if(views_[view].at<uchar>(wr,wc) < *p_center ) //compare pixel values in the neighborhood
                  p_census_val = p_census_val | 0x1;

              }
              shift_count ++;
            }
          }
          *p_census = p_census_val;
        }
      }
    }

    cout <<"\nFinding Hamming Distance" <<endl;
    
    // Loop over image rows
    for(int r = window_height_/2 + 1; r < height_ - window_height_/2 - 1; r++)
    {
    
      // Loop over image columns
      for(int c = window_width_/2 + 1; c < width_ - window_width_/2 - 1; c++)
      {
        
        // Loop over disparity values
        for(int d=0; d<disparity_range_; d++)
        {
          long cost = 0;
          // Loop over window rows
          for(int wr = r - window_height_/2; wr <= r + window_height_/2; wr++)
          {
            
            // Pointers to current rows in census images
            uchar *p_left = census_img[0].ptr<uchar>(wr),
                  *p_right = census_img[1].ptr<uchar>(wr);


            int wc = c - window_width_/2;
            p_left += wc;
            p_right += wc + d;



            const uchar out_val = census_img[1].at<uchar>(wr, width_ - window_width_/2 - 1);


            // Loop over window columns
            for(; wc <= c + window_width_/2; wc++, p_left++, p_right++)
            {
              uchar census_left, census_right, m_census_left, m_census_right;
              census_left = *p_left;
              if (c+d < width_ - window_width_/2)
              {
                census_right= *p_right;

              }

              else
              {
                census_right= out_val;
              }


              cost += ((hamLut[census_left][census_right]));
            }
          }
          cost_[r][c][d]=cost;
        }
      }
    }
  }
```

## compute_path_cost()
The objective of the **compute_path_cost()** function is to calculate the cost associated with a given path for a particular pixel **p** (specified by its coordinates **cur_x** and **cur_y**). The cost should be computed for all possible disparities **d** within the range of 0 to **disparity_range_** - 1. The calculated costs should be stored in a tensor named **path_cost_[cur_path][cur_y][cur_x][d]**.

**Input parameters**: **direction_y**, **direction_x**, specify the direction of the path. **cur_y**, **cur_x**, coordinates of pixel **p**. **cur_path**, index of the current path.

```C++
void SGM::compute_path_cost(int direction_y, int direction_x, int cur_y, int cur_x, int cur_path)
  {
    unsigned long prev_cost, best_prev_cost, no_penalty_cost, penalty_cost, 
                  small_penalty_cost, big_penalty_cost;

    //////////////////////////// Code to be completed (1/4) /////////////////////////////////
    // Complete the compute_path_cost() function that, given: 
    // i) a single pixel p defined by its coordinates cur_x and cur_y; 
    // ii) a path with index cur_path (cur_path=0,1,..., PATHS_PER_SCAN - 1, a path for 
    //     each direction), and;
    // iii) the direction increments direction_x and direction_y associated with the path 
    //      with index cur_path (that are the dx,dy increments to move along the path 
    //      direction, both can be -1, 0, or 1), 
    // should compute the path cost for p for all the possible disparities d from 0 to 
    // disparity_range_ (excluded, already defined). The output should be stored in the 
    // tensor (already allocated) path_cost_[cur_path][cur_y][cur_x][d], for all possible d.
    /////////////////////////////////////////////////////////////////////////////////////////

    // if the processed pixel is the first:
    if(cur_y == pw_.north || cur_y == pw_.south || cur_x == pw_.east || cur_x == pw_.west)
    {
      //Please fill me!
    }

    else
    {
      //Please fill me!
    }
    
    
    /////////////////////////////////////////////////////////////////////////////////////////
  }
```

- **aggregation()**: Aggregates the costs along different paths.

```C++
void SGM::aggregation()
  {
    
    //for all defined paths
    for(int cur_path = 0; cur_path < PATHS_PER_SCAN; ++cur_path)
    {

      //////////////////////////// Code to be completed (2/4) /////////////////////////////////
      // Initialize the variables start_x, start_y, end_x, end_y, step_x, step_y with the 
      // right values, after that uncomment the code below
      /////////////////////////////////////////////////////////////////////////////////////////

      int dir_x = paths_[cur_path].direction_x;
      int dir_y = paths_[cur_path].direction_y;
      
      int start_x, start_y, end_x, end_y, step_x, step_y;
      
//      for(int y = start_y; y != end_y ; y+=step_y)
//      {
//        for(int x = start_x; x != end_x ; x+=step_x)
//        {
//          compute_path_cost(dir_y, dir_x, y, x, cur_path);
//        }
//      }
      
      /////////////////////////////////////////////////////////////////////////////////////////
    }
    
    float alpha = (PATHS_PER_SCAN - 1) / static_cast<float>(PATHS_PER_SCAN);
    //aggregate the costs
    for (int row = 0; row < height_; ++row)
    {
      for (int col = 0; col < width_; ++col)
      {
        for(int path = 0; path < PATHS_PER_SCAN; path++)
        {
          unsigned long min_on_path = path_cost_[path][row][col][0];
          int disp =  0;

          for(int d = 0; d<disparity_range_; d++)
          {
            aggr_cost_[row][col][d] += path_cost_[path][row][col][d];
            if (path_cost_[path][row][col][d]<min_on_path)
              {
                min_on_path = path_cost_[path][row][col][d];
                disp = d;
              }

          }
          inv_confidence_[row][col] += (min_on_path - alpha * cost_[row][col][disp]);

        }
      }
    }

  }

```

- **compute_disparity()**: Computes the disparity map by first calculating costs, then aggregating them, and finally estimating disparities.

```C++
void SGM::compute_disparity()
  {
      calculate_cost_hamming();
      aggregation();
      disp_ = Mat(Size(width_, height_), CV_8UC1, Scalar::all(0));
      int n_valid = 0;
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

              // If the following condition is true, the disparity at position (row, col) has a good confidence
              if (inv_confidence_[row][col] > 0 && inv_confidence_[row][col] <conf_thresh_)
              {
                //////////////////////////// Code to be completed (3/4) /////////////////////////////////
                // Since the disparity at position (row, col) has a good confidence, it can be added 
                // togheter with the corresponding unscaled disparity from the right-to-left initial 
                // guess mono_.at<uchar>(row, col) to the pool of disparity pairs that will be used 
                // to estimate the unknown scale factor.    
                /////////////////////////////////////////////////////////////////////////////////////////

                
                
                
                
                
                
                /////////////////////////////////////////////////////////////////////////////////////////
              }

              disp_.at<uchar>(row, col) = smallest_disparity*255.0/disparity_range_;

          }
      }

      //////////////////////////// Code to be completed (4/4) /////////////////////////////////
      // Using all the disparity pairs accumulated in the previous step, 
      // estimate the unknown scaling factor and scale the initial guess disparities 
      // accordingly. Finally,  and use them to improve/replace the low-confidence SGM 
      // disparities.
      /////////////////////////////////////////////////////////////////////////////////////////

      
      
      
      
      
      
      
      /////////////////////////////////////////////////////////////////////////////////////////

  }
```

- **compute_mse()**: Computes the Mean Squared Error (MSE) between the computed disparity map and the ground truth.

```C++
float SGM::compute_mse(const cv::Mat &gt)
  {
    cv::Mat1f container[2];
    cv::normalize(gt, container[0], 0, 85, cv::NORM_MINMAX);
    cv::normalize(disp_, container[1], 0, disparity_range_, cv::NORM_MINMAX);

    cv::Mat1f  mask = min(gt, 1);
    cv::multiply(container[1], mask, container[1], 1);
    float error = 0;
    for (int y=0; y<height_; ++y)
    {
      for (int x=0; x<width_; ++x)
      {
        float diff = container[0](y,x) - container[1](y,x);
        error+=(diff*diff);
      }
    }
    error = error/(width_*height_);
    return error;
  }
```

- **save_disparity()**: Saves the computed disparity map to an image file.

```C++
void SGM::save_disparity(char* out_file_name)
  {
    imwrite(out_file_name, disp_);
    return;
  }
```





























	
