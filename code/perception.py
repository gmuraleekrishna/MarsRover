import numpy as np
import cv2


# Define a function to perform a perspective transform
# I've used the example grid image above to choose source points for the
# grid cell in front of the rover (each grid cell is 1 square meter in the sim)
def perspect_transform(img):
	img_size = (img.shape[1], img.shape[0])
	# Define calibration box in source (actual) and destination (desired) coordinates
	# These source and destination points are defined to warp the image
	# to a grid where each 10x10 pixel square represents 1 square meter
	dst_size = 5
	# Set a bottom offset to account for the fact that the bottom of the image
	# is not the position of the rover but a bit in front of it
	# this is just a rough guess, feel free to change it!
	bottom_offset = 6
	src = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
	dst = np.float32([[img_size[0] / 2 - dst_size, img_size[1] - bottom_offset],
	                  [img_size[0] / 2 + dst_size, img_size[1] - bottom_offset],
	                  [img_size[0] / 2 + dst_size, img_size[1] - 2 * dst_size - bottom_offset],
	                  [img_size[0] / 2 - dst_size, img_size[1] - 2 * dst_size - bottom_offset],
	                  ])

	M = cv2.getPerspectiveTransform(src, dst)
	warped = cv2.warpPerspective(img, M, img_size)  # keep same size as input image
	return warped


# Apply the above functions in succession
# Identify pixels below the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, low_thresh=(0, 0, 0), high_thresh=(255, 255, 255)):
	# Create an array of zeros same xy size as img, but single channel
	color_select = np.zeros_like(img[:, :, 0])
	# Require that each pixel be above all thre threshold values in RGB
	# above_thresh will now contain a boolean array with "True"
	# where threshold was met
	thresh_img = (img[:, :, 0] >= low_thresh[0]) \
	             & (img[:, :, 1] >= low_thresh[1]) \
	             & (img[:, :, 2] >= low_thresh[2]) \
	             & (img[:, :, 0] <= high_thresh[0]) \
	             & (img[:, :, 1] <= high_thresh[1]) \
	             & (img[:, :, 2] <= high_thresh[2])
	# Index the array of zeros with the boolean array and set to 1
	color_select[thresh_img] = 1
	# Return the binary image
	return color_select


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img, limit=80):
	# Identify nonzero pixels
	ypos, xpos = binary_img.nonzero()
	# Calculate pixel positions with reference to the rover position being at the
	# center bottom of the image.
	y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
	x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
	dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
	x_pixel, y_pixel = x_pixel[dist < limit], y_pixel[dist < limit]
	return x_pixel, y_pixel


# Define a function to map rover space pixels to world space
def pix_to_world(dist, angles, x_rover, y_rover, yaw_rover):
	# Map pixels from rover space to world coords
	pix_angles = angles + (yaw_rover * np.pi / 180)
	# Assume a worldmap size of 200 x 200
	world_size = 200
	# Assume factor of 10 scale change between rover and world space
	scale = 10
	x_pix_world = np.clip(np.int_((dist / scale * np.sin(pix_angles)) + x_rover), 0, world_size - 1)
	y_pix_world = np.clip(np.int_((dist / scale * np.cos(pix_angles)) + y_rover), 0, world_size - 1)
	return x_pix_world, y_pix_world


def to_polar_coords(xpix, ypix):
	# Calculate distance to each pixel
	dist = np.sqrt(xpix ** 2 + ypix ** 2)
	# Calculate angle using arctangent function
	angles = np.arctan2(ypix, xpix)
	return dist, angles


# Apply the above functions in succession
def calc_forward_dist(path_dists, path_angles):
	abs_angles = np.absolute(path_angles / sum(path_angles))
	idx = np.abs(abs_angles).argmin()
	return path_dists[idx]


def perception_step(Rover):
	# Perform perception steps to update Rover()
	# TODO:

	# 5) Convert map image pixel values to rover-centric coords
	# 6) Convert rover-centric pixel values to world coordinates
	# 7) Update Rover worldmap (to be displayed on right side of screen)
	# Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
	#          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
	#          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

	# 8) Convert rover-centric pixel positions to polar coordinates
	# Update Rover pixel distances and angles
	# Rover.nav_dists = rover_centric_pixel_distances
	# Rover.nav_angles = rover_centric_angles

	map_img = perspect_transform(Rover.img)
	path_thres_img = color_thresh(map_img, low_thresh=(160, 160, 160))
	rock_thres_img = color_thresh(map_img, low_thresh=(140, 120, 0), high_thresh=(255, 230, 80))
	obstacle_thres_img = color_thresh(map_img, high_thresh=(160, 160, 160))

	Rover.vision_image[:, :, 2] = path_thres_img * 255
	Rover.vision_image[:, :, 1] = rock_thres_img * 255
	Rover.vision_image[:, :, 0] = obstacle_thres_img * 255

	path_xpix, path_ypix = rover_coords(path_thres_img)  # Convert to rover-centric coords
	rock_xpix, rock_ypix = rover_coords(rock_thres_img)  # Convert to rover-centric coords
	obst_xpix, obst_ypix = rover_coords(obstacle_thres_img)  # Convert to rover-centric coords

	path_dists, path_angles = to_polar_coords(path_xpix, path_ypix)  # Convert to polar coords
	rock_dist, rock_angles = to_polar_coords(rock_xpix, rock_ypix)  # Convert to polar coords
	obs_dist, obs_dist = to_polar_coords(obst_xpix, obst_ypix)  # Convert to polar coords

	navigable_x_world, navigable_y_world = pix_to_world(path_dists, path_angles, Rover.pos[0], Rover.pos[1], Rover.yaw)
	rock_x_world, rock_y_world = pix_to_world(rock_dist, rock_angles, Rover.pos[0], Rover.pos[1], Rover.yaw)
	obstacle_x_world, obstacle_y_world = pix_to_world(obs_dist, obs_dist, Rover.pos[0], Rover.pos[1], Rover.yaw)

	if len(path_angles) > 0:
		Rover.dist_to_obstacle = calc_forward_dist(path_dists, path_angles)

	if (Rover.pitch < 1 or Rover.pitch > 359) and (Rover.roll < 1 or Rover.roll > 359):
		Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] = 255
		Rover.worldmap[rock_y_world, rock_x_world, 1] = 255
		Rover.worldmap[navigable_y_world, navigable_x_world, 2] = 255
		# remove overlap mesurements
		nav_pix = Rover.worldmap[:, :, 2] > 0
		Rover.worldmap[nav_pix, 0] = 0
		# clip to avoid overflow
		Rover.worldmap = np.clip(Rover.worldmap, 0, 255)

	Rover.nav_dists = path_dists
	Rover.nav_angles = path_angles

	Rover.samples_dists = rock_dist
	Rover.samples_angles = rock_angles

	Rover.nav_area = path_thres_img.sum()

	return Rover
