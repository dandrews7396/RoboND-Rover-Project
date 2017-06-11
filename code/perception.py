import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, lwr_thresh=(113, 155, 155), upr_thresh = (255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # thresh will now contain a boolean array with "True"
    # where threshold was met
    thresh = (img[:,:,0] > lwr_thresh[0]) \
                & (img[:,:,1] > lwr_thresh[1]) \
                & (img[:,:,2] > lwr_thresh[2]) \
                & (img[:,:,0] < upr_thresh[0]) \
                & (img[:,:,1] < upr_thresh[1]) \
                & (img[:,:,2] < upr_thresh[2])

    # Index the array of zeros with the boolean array and set to 1
    select[thresh] = 1
    # Return the binary image
    return select

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
    nav = color_thresh(warped)
    obs = color_thresh(warped, (10,10,10), (155, 155, 155))
    ## Apply majorly open threshold for yellow, because old 'good' values just don't seem to work.
    rocks = color_thresh(warped, (128, 110, 0), (190, 160, 40))
    # Update Rover.vision_image (this will be displayed on left side of screen)
    rover_obs = color_thresh(img, (128, 110, 0), (190, 160, 40))
    rover_rocks = color_thresh(img, (10,10,10), (155, 155, 155))
    rover_nav = color_thresh(img)
    Rover.vision_image[:,:,1] = rover_obs * 255
    Rover.vision_image[:,:,0] = rover_rocks * 255
    Rover.vision_image[:,:,2] = rover_nav * 255

    # Convert map image pixel values to rover-centric coords
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
    # Update Rover worldmap (to be displayed on right side of screen), only when pitch
    # and roll are not extreme. This will ensure only quality data is added to worldmap
    if np.abs(Rover.pitch) > 1 or np.abs(Rover.roll) > 1:
        Rover.worldmap[yobs_world, xobs_world, 0] += 1
        Rover.worldmap[yrock_world, xrock_world, 1] += 1
        Rover.worldmap[ynav_world, xnav_world, 2] += 1

    # Convert rover-centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(xnav_pix, ynav_pix)
    Rover.obs_dists, Rover.obs_angles = to_polar_coords(xobs_pix, yobs_pix)
    Rover.rock_dists, Rover.rock_angles = to_polar_coords(xrock_pix, yrock_pix)
    Rover.front_space = [Rover.nav_dists[i] for i in range(Rover.nav_angles.shape[0]) if
                                     (Rover.steer-0.5) <= (Rover.nav_angles[i] * 180/np.pi) <= (Rover.steer+0.5)]
    
    # Update Rover pixel distances and angles
    #Rover.nav_dists = rover_centric_pixel_distances
    #Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover
