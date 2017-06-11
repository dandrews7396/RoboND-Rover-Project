import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    theta_mean = np.mean(Rover.nav_angles * 180/np.pi)
    theta_median = np.median(Rover.nav_angles * 180/np.pi)
    # Lest's look for rock samples first
##    if Rover.rock_angles is not None:
##        if Rover.mode == 'forward':
##            rock_sample_dist = np.sqrt(
##        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            last_known_x, last_known_y = Rover.pos
            #time_at_lkp = time.time()
            # Check the extent of navigable terrain
            if max(Rover.front_space) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle if roll is not harsh
                # This should prevent fishtailing, and improve fidelity  and np.abs(Rover.roll) <0.5
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Figure out which way to steer
                if theta_mean > 0:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                          theta_mean+(theta_mean/2))
                elif theta_mean < 0:
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                          theta_mean+(theta_mean/2))
                else:
                    Rover.steer = 0
                # Set steering to average angle clipped to the range +/- 15
##                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
##                nav_dist_list = [x for x in np.nditer(Rover.nav_dists)]
##                nav_ang_list = [x for x in np.nditer(Rover.nav_angles)]
                # Check the free space in rough direction of steer angle
                
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif max(Rover.front_space) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                if Rover.near_sample and not Rover.vel:
                    Rover.send_pickup = True
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.front_space) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Figure out which way to steer
                    if theta_mean > 0:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                              theta_mean+(theta_mean/2))
                    elif theta_mean < 0:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                              theta_mean+(theta_mean/2))
                    else:
                        Rover.steer = -15
                        
##                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
##                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                else:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Figure out which way to steer
                    if theta_mean > 0:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                              theta_mean+(theta_mean/2))
                    elif theta_mean < 0:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), (theta_mean/2),
                                              theta_mean+(theta_mean/2))
                    else:
                        Rover.steer = 0
##                    # Set steer to mean angle
##                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

