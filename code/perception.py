import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
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
    
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])

    ## (gold is 255, 215, 0), cv2 works in BGR
    rock_thresh = (img[:,:,0] > 0) \
                & (img[:,:,1] > 215) \
                & (img[:,:,2] < 255)

    # Index the array of zeros with the boolean array and set to 1
    nav_select[above_thresh] = 1
    obs_select[below_thresh] = 1
    rock_select[rock_thresh] = 1

    # Return the binary images
    return nav_select, obs_select, rock_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rads = yaw * np.pi/180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rads) - ypix * np.sin(yaw_rads)
    ypix_rotated = xpix * np.sin(yaw_rads) + ypix *np.cos(yaw_rads)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot/scale))
    ypix_translated = np.int_(ypos + (ypix_rot/scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover() 
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img
    # Define scale and world size for image:map relationship  
    scale = 10
    world_size = 200
    # Define image size for ease of use later
    img_size = (img.shape[1], img.shape[0])
    # Define source and destination points for perspective transform
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[img_size[0]/2 - dst_size, img_size[1] - bottom_offset],
                      [img_size[0]/2 + dst_size, img_size[1] - bottom_offset],
                      [img_size[0]/2 + dst_size, img_size[1] - 2*dst_size - bottom_offset], 
                      [img_size[0]/2 - dst_size, img_size[1] - 2*dst_size - bottom_offset],
                      ])
    # Apply perspective transform
    warped = perspect_transform(img, src, dst)
    # Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav, obs, rocks = color_thresh(warped)
    # Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obs
    Rover.vision_image[:,:,1] = rocks
    Rover.vision_image[:,:,2] = nav

    # 5) Convert map image pixel values to rover-centric coords
    xnav_pix, ynav_pix = rover_coords(nav)
    xobs_pix, yobs_pix = rover_coords(obs)
    xrock_pix, yrock_pix = rover_coords(rocks)
    # Convert rover-centric pixel values to world coordinates
    xnav_world, ynav_world = pix_to_world(xnav_pix, ynav_pix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw,
                                          world_size, scale)
    xobs_world, yobs_world = pix_to_world(xobs_pix, yobs_pix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw,
                                          world_size, scale)
    xrock_world, yrock_world = pix_to_world(xrock_pix, yrock_pix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw,
                                          world_size, scale)
    # Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[yobs_world, xobs_world, 0] += 1
    Rover.worldmap[yrock_world, xrock_world, 1] += 1
    Rover.worldmap[ynav_world, xnav_world, 2] += 1

    # Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xnav_pix, ynav_pix)
    # Update Rover pixel distances and angles
    #Rover.nav_dists = rover_centric_pixel_distances
    #Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover
