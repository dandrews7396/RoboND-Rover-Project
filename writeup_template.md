## Project: Search and Sample Return


---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./output/nav_threshed.jpg
[image2]: ./output/obs_threshed.jpg
[image3]: ./output/rock_threshed.jpg 
[image4]: ./calibration_images/example_grid1.jpg
[image5]: ./output/rover_coords_plot.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I shall consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This Writeup shall satisfy the requirement.

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

After the initial runthrough of the notebook, I set about altering the function `color_thresh()` to output binary images with thresholds applied to separate out data for obstacles and rock samples, as well as navigable terrain which I stored as `above_thresh`. In doing this I identified that, as anything that is not navigable terrain must be an obstacle, then the threshold for obstacles should therefore be the inverse of that for a suitable path. I stored the result of that threshold to the variable `below_thresh`. The outputs for both navigable terrain and obstacles can be compared below.

![alt text][image1]
![alt text][image2]

The next step was to apply a threshold for detecting samples. To aid me in this, I used [color-hex](www.color-hex.com) to identify a [good shade of yellow](http://www.color-hex.com/color/ffdd23) that produced the desireable results from a new threshold, which I stored to the variable `rock_thresh`. The output of this threshold can be seen below.

![alt text][image3]

I had considered applying these thresholds as seperate function calls, but thought it just as effective to apply them in one, single function, and then return them as an iterable. I include here, my version of `colour_thresh()`:

```
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    nav_select = np.zeros_like(img[:,:,0])
    obs_select = np.zeros_like(img[:,:,0])
    rock_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # That which isn't a path, must be an obstacle
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])

    ## (good yellow is 255, 221, 35) but in cv2 bgr, so
    rock_thresh = (img[:,:,0] > 35) \
                & (img[:,:,1] > 221) \
                & (img[:,:,2] < 255)

    # Index the array of zeros with the boolean array and set to 1
    nav_select[above_thresh] = 1
    obs_select[below_thresh] = 1
    rock_select[rock_thresh] = 1

    # Return the binary images
    return nav_select, obs_select, rock_select
```

#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

In completing the `process_image()` funtion, I first had to define calibration co-ordinates to be used for the perspective transform. These were identified using an image, taken from the rover's view, with the grid option applied to the terrain.

![alt text][image4]

Using a matplotlib output, I was able to identify pixel co-odrinates for the corners of a $$1m^2$$ box in the image, these became my source co-ordinates. I then identified the positions that I expected those points to appear on a top-down view image, that was to be the same size as the original. These were to be my destination co-ordinates and, along with the source, they were defined as:

```
src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
dst = np.float32([[img_size[0]/2 - dst_size, img_size[1] - bottom_offset],
                  [img_size[0]/2 + dst_size, img_size[1] - bottom_offset],
                  [img_size[0]/2 + dst_size, img_size[1] - 2*dst_size - bottom_offset], 
                  [img_size[0]/2 - dst_size, img_size[1] - 2*dst_size - bottom_offset],
                  ])
```

These were then passed, together with the original image, to the `perspect_transform()` function, and stored in the variable `warped`. This warped image was then passed to the, previously described, `color_threshed()` function which, as explained earlier, returns three images; One each for navigable terrain, obstacles and rock samples.

These three images are then run through the `rover_coords()` function. This function initially identifies nonzero pixels within the image, and then tranlates these onto a plot with reference to the position of the rover being at the center of the page, except that that centre is now on the Y-axis. This is made more clear in the next image.

![alt text][image5]


![alt text][image2]
### Autonomous Navigation and Mapping

#### 3. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]


