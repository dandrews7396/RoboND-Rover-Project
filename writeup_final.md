## Project: Search and Sample Return


---


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

These three images are then run through the `rover_coords()` function. This function initially identifies nonzero pixels within the image, and then tranlates these onto a plot with reference to the position of the rover being at the center of the page, except that that centre is now on the Y-axis. This process is made more clear in the next image.

![alt text][image5]

The next step was to process the results of the `rover_coords()` function, and turn them into co-ordinates that I could plot onto the `worldmap`. In order to achieve this, I made use of the `pix_to_world()` function. This made use of to further functions, `rotate_pix()` and `translate_pix()`, together they perform a 2-D rotational transformation, using matrix multiplication, and then a standard transformation.

The resulting co-ordinates for the navigable path, obstacles and samples are then overlayed onto `worldmap` by incrementing the resulting pionts, on each of the respective layers by one.

Finally, the output images are processed and are returned.


### Autonomous Navigation and Mapping

#### 3. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

In filling out `perception_step()`, I essentially used the same code as `process_image()`. It seemed to work fairly well in the notebook, so I used this as a basis to build from. I realised that the rover was speeding around quite a lot, and had a tendency to swerve around. Because of this, the rover failed to detect if it was coming to a dead-end. However, the rover was picking up most of the rock samples, so that showed that I had the thresholds right. 

Based on this evidence, I decided to add a condition that stopped the rover adding data to the world map in high pitch and roll conditions. I wasn't sure what values to use at first but with a little trial and error, I settled on this:

```
if np.abs(Rover.pitch) < 0.4 and np.abs(Rover.roll) < 0.9:
        Rover.worldmap[yobs_world, xobs_world, 0] += 1
        Rover.worldmap[yrock_world, xrock_world, 1] += 1
        Rover.worldmap[ynav_world, xnav_world, 2] += 1
```

I chose to use absolute absolute values, as this would mean I wouldn't have to write additional logic statements to account for negative numbers. I settled on these figures because I found that if they were any more restrictive, then no data would be recorded unless crawling along.

Despite these imrovements, I still wasn't happy with the fidelity my rover was achieving. For this reason, I decided I would need to reduce the number of occasions that I wasn't adding data to the world map. This meant changing `decision_step`. I hypothesised that if I were to base the acceleration condition on roll, then this would at least stop the rover from swerving so violently and in turn, provide it with more opportunities to accept data to `worldmap`. So I altered the code to read:

```
if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle if roll is not harsh
                # This should prevent fishtailing, and improve fidelity
                if Rover.vel < Rover.max_vel and np.abs(Rover.roll) <0.5:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
```

This is something that I have also witnessed in aviation, when the Lynx helicopter flies over a certain airspeed, it starts to experience lateral 'fish-tailing'. My rover appeared to be behaving in a very similar manner. The effect on fidelity and mapping was very positive.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

On initially trying to run the rover autonomously, I encountered a massive list of ecxeptions. I traced it to the `update_rover()` function in `supporting_functions.py`. I solved this by changing the `split()` condition when trying to read position data.

From this:
```
samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(';')])
samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(';')])
...
# The current position of the rover
Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(';')]
```
To this
```
samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(',')])
samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(',')])
...
# The current position of the rover
Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(',')]
```


I decided it would be prudent to test my rover over a number of tries. I knew fidelity wouldn't be a problem, so I decided I would run my test until I achieved 40% mapping and at least 1 sample detected. I also chose to run these over different image qualities. The results were as follows:

|Quality  |Time   |Mapped %|Fidelity %|Samples|
|:-------:|------:|:------:|:--------:|:-----:|
|Fastest  |149s   |40      |84        |1      |
|Good     |153s   |40      |82        |2      |
|Fantastic|241s   |58      |78        |1      |

As you can see, the fidelity for these results is quite high. I expected that to drop away as the image quality increased, as I thought it may become harder to pick out the samples and distinguish between obstacle and shadow, but was pleased to see it remain near 80%. I believe that the other variables are down to chance in a lot of cases, as they depend a lot on the random placement of the samples and which direction the rover is pointing in at the start point. But I noted that whenever a sample appeared in the main view, it was quickly identified. Given more time, I would like to have been able to test this theory a lot more, perhaps even changing start position and other variables, if possible.

I think, if I had more experience with computer vision I would probably have done better with distinguishing between features. I know there is a better way to evaluate the distance to objects, but I'm not sure how to implement it at this time. Really I think the only things I have really lacked over the course of this project are experience and time. But I see this as a positive because this is my first time on a course like this. I have a good grasp on the basics of coding, and I have already learnt a lot more.

I would like to have been able to implement the sample pick ups in a more effective way, and I think that there is a lot more refinement I could have done on the rovers' decision making. I will look forward to revisiting this project once I have developed my skillset a little more.



