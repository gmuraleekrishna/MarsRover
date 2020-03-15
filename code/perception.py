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
	dst = np.float32([[img_size[0] / 2 - dst_size, img_size[1] - bottom_offset], \
	                  [img_size[0] / 2 + dst_size, img_size[1] - bottom_offset], \
	                  [img_size[0] / 2 + dst_size, img_size[1] - 2 * dst_size - bottom_offset], \
	                  [img_size[0] / 2 - dst_size, img_size[1] - 2 * dst_size - bottom_offset],
	                  ])

	M = cv2.getPerspectiveTransform(src, dst)
	warped = cv2.warpPerspective(img, M, img_size)  # keep same size as input image
	return warped


# Apply the above functions in succession
# Identify pixels below the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
	# Create an array of zeros same xy size as img, but single channel
	color_select = np.zeros_like(img[:, :, 0])
	# Require that each pixel be above all thre threshold values in RGB
	# above_thresh will now contain a boolean array with "True"
	# where threshold was met
	above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
	               & (img[:, :, 1] > rgb_thresh[1]) \
	               & (img[:, :, 2] > rgb_thresh[2])
	# Index the array of zeros with the boolean array and set to 1
	color_select[above_thresh] = 1
	# Return the binary image
	return color_select


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
	# Identify nonzero pixels
	ypos, xpos = binary_img.nonzero()
	# Calculate pixel positions with reference to the rover position being at the
	# center bottom of the image.
	y_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
	x_pixel = (xpos - binary_img.shape[0]).astype(np.float)
	return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_radial_coords(x_pixel, y_pixel):
	# Convert (x_pixel, y_pixel) to (distance, angle)
	# in polar coordinates in rover space
	# Calculate distance to each pixel
	dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
	# Calculate angle away from vertical for each pixel
	angles = np.arctan2(x_pixel, y_pixel)
	return dist, angles


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
def perception_step(Rover):
	map_img = perspect_transform(Rover.img)
	thres_img = color_thresh(map_img)
	xpix, ypix = rover_coords(thres_img)  # Convert to rover-centric coords
	Rover.nav_dists, Rover.nav_angles = to_polar_coords(xpix, ypix)  # Convert to polar coords
	# Rover.avg_angle = np.mean(angles)  # Compute the average angle
	x_world, y_world = pix_to_world(Rover.nav_dists, Rover.nav_angles, Rover.pos[0], Rover.pos[1], Rover.yaw)
	# Update the map
	Rover.worldmap[x_world, y_world, 2] += 10
	return Rover
