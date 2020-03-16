import numpy as np


# This is basically a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
	# Check if we have vision data to make decisions with
	if Rover.nav_angles is not None:
		if Rover.mode == 'sampling':
			if Rover.vel != 0:
				Rover.brake = 10
				if len(Rover.samples_dists) == 0:
					Rover.mode = 'forward'

		elif Rover.mode == 'stuck' and len(Rover.samples_dists) == 0:
			if Rover.vel != 0:
				Rover.brake = 10
			Rover.steer = -15
			Rover.throttle = 0
			Rover.brake = 0
			if Rover.nav_area > 650:
				Rover.mode = 'forward'
		elif Rover.mode == 'forward':
			if 650 > Rover.nav_area > 10:
				Rover.throttle = 0
				Rover.brake = 10
				Rover.steer = 0
				Rover.mode = 'stuck'
			if len(Rover.samples_dists) > 0 and np.min(Rover.samples_dists) < (5 * Rover.vel):
				Rover.mode = 'sampling'
				Rover.throttle = 0
				Rover.brake = 10
				Rover.steer = 0

			elif len(Rover.nav_angles) >= Rover.stop_forward:
				# If mode is forward, navigable terrain looks good
				# Except for start, if stopped means stuck.
				# Alternates between stuck and forward modes
				if len(Rover.samples_angles) > 0:
					drive_angles = Rover.samples_angles
					drive_distance = np.min(Rover.samples_dists)
				else:
					drive_angles = Rover.nav_angles
					drive_distance = Rover.dist_to_obstacle

					# Set throttle value to throttle setting
				Rover.throttle = np.clip(drive_distance * 0.005 - Rover.vel * 0.2, 0, 2)
				Rover.brake = 0
				# Set steering to average angle clipped to the range +/- 15
				Rover.steer = np.clip(np.mean(drive_angles * 180 / np.pi), -15, 15)
				# Hug left wall by setting the steer angle slightly to the left
				# Rover.steer = np.clip(np.mean((Rover.nav_angles + offset) * 180 / np.pi), -15, 15)

			# If there's a lack of navigable terrain pixels then go to 'stop' mod
			# If we're already in "stuck". Stay here for 1 sec
			else:
				Rover.throttle = 0
				# Release the brake to allow turning
				Rover.brake = 0
				# Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
				# Since hugging left wall steering should be to the right:
				Rover.steer = 0
				Rover.mode = 'stop'
		# If we're already in "stop" mode then make different decisions
		elif Rover.mode == 'stop':
			# If we're in stop mode but still moving keep braking
			if Rover.vel > 0.2:
				Rover.throttle = 0
				Rover.brake = 10
				Rover.steer = 0
			if len(Rover.nav_angles) > 0:
				Rover.mode = 'forward'

	# Just to make the rover do something
	# even if no modifications have been made to the code
	else:
		Rover.throttle = Rover.throttle_set
		Rover.steer = 0
		Rover.brake = 0

	# If in a state where want to pickup a rock send pickup command
	if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
		Rover.send_pickup = True
	print('State', Rover.mode)
	return Rover
