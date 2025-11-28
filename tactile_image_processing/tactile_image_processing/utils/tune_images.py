"""
Author: Martijn Brummelhuis
"""
import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from tactile_image_processing.image_transforms import process_image
from tactile_image_processing.simple_sensors import RealSensor

def setup_sensor(exp=-7):
    bbox_dict = {
        'mini': (320-160,    240-160+25, 320+160,    240+160+25),
        'midi': (320-220+10, 240-220-20, 320+220+10, 240+220-20),
        'aerial-A': (95, 40, 535, 480)
    }
    sensor_type = 'aerial-A'
    
    sensor_image_params = {
        'type': sensor_type,
        'source': 4,
        'exposure': exp,
        'gray': True,
        'bbox': bbox_dict[sensor_type]
    }
    sensor = RealSensor(sensor_image_params)
    
    return sensor

# Setup sensor
sensor = setup_sensor()

# Take an image
image_rgb = sensor.read()

# Define update function for the bounding box
def update_image(thresh1, thresh2, cmr, x_min, x_max, y_min, y_max, set_exposure):
    
    # Setup sensor
    sensor.set_exposure(new_exposure=set_exposure)

    # Take a new image (in case of exposure change)
    image_rgb = sensor.read()
    
    # Ensure the sliders do not cross
    x_min, x_max = sorted([x_min, x_max])
    y_min, y_max = sorted([y_min, y_max])

    # Crop the image according to the bounding box sliders
    cropped_image = image_rgb[int(y_min):int(y_max), int(x_min):int(x_max)]
    
    reprocessed_image = process_image(cropped_image, thresh=[thresh1, thresh2], circle_mask_radius=cmr)
    
    # Clear the current plot
    ax.clear()
    
    # Display the updated image
    ax.imshow(reprocessed_image, cmap='gray', vmin=0, vmax=255)
    ax.set_title(f'Thresh: {thresh1, thresh2}, Mask radius: {cmr}, BBox: [{x_min},{x_max},{y_min},{y_max}]')
    
    # Calculate and display current resolution
    current_resolution_x = x_max - x_min
    current_resolution_y = y_max - y_min
    ax.text(0.05, 0.95, f'Resolution: {current_resolution_x} x {current_resolution_y}',
            transform=ax.transAxes, fontsize=10, color='black', backgroundcolor='white', va='top')
 
    plt.draw()

# Create the sliders for thresholding and circle mask radius
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.4, bottom=0.35)  # Adjust layout for sliders
ax.imshow(image_rgb, cmap='gray')
ax.set_title('Original Image')

# Create sliders for xmin, xmax, ymin, ymax on the left side
ax_xmin = plt.axes([0.05, 0.5, 0.02, 0.3], facecolor='lightgoldenrodyellow')  # xmin
ax_xmax = plt.axes([0.10, 0.5, 0.02, 0.3], facecolor='lightgoldenrodyellow')  # xmax
ax_ymin = plt.axes([0.15, 0.5, 0.02, 0.3], facecolor='lightgoldenrodyellow')  # ymin
ax_ymax = plt.axes([0.20, 0.5, 0.02, 0.3], facecolor='lightgoldenrodyellow')  # ymax

# Create sliders for threshold and mask radius on the bottom
ax_thresh1 = plt.axes([0.25, 0.25, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_thresh2 = plt.axes([0.25, 0.20, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_cmr = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_exposure = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor='lightgoldenrodyellow')


# Define the min/max for the bounding box sliders (depends on image dimensions)
image_height, image_width = image_rgb.shape[:2]
xmin_slider = Slider(ax_xmin, 'X Min', 0, image_width, valinit=0, valstep=5, orientation='vertical')
xmax_slider = Slider(ax_xmax, 'X Max', 0, image_width, valinit=image_width, valstep=5, orientation='vertical')
ymin_slider = Slider(ax_ymin, 'Y Min', 0, image_height, valinit=0, valstep=5, orientation='vertical')
ymax_slider = Slider(ax_ymax, 'Y Max', 0, image_height, valinit=image_height, valstep=5, orientation='vertical')

# Create the other sliders for thresholding and circle mask radius
thresh1_slider = Slider(ax_thresh1, 'Threshold 1 blocksize', 3, 201, valinit=61, valstep=2)
thresh2_slider = Slider(ax_thresh2, 'Threshold 2 constant', -201, 99, valinit=5)
cmr_slider = Slider(ax_cmr, 'Circle Mask Radius', 10, 400, valinit=200, valstep=1)
exposure_slider = Slider(ax_exposure, 'Exposure', -100, 1000, valinit=-7, valstep=.1)

# Update the plot when any slider is changed
def on_slider_change(val):
    # Ensure min sliders don't exceed max sliders
    x_min = min(xmin_slider.val, xmax_slider.val)
    x_max = max(xmin_slider.val, xmax_slider.val)
    y_min = min(ymin_slider.val, ymax_slider.val)
    y_max = max(ymin_slider.val, ymax_slider.val)

    update_image(int(thresh1_slider.val), thresh2_slider.val, cmr_slider.val,
                 x_min, x_max, y_min, y_max, set_exposure=exposure_slider.val)

# Link the slider updates
thresh1_slider.on_changed(on_slider_change)
thresh2_slider.on_changed(on_slider_change)
cmr_slider.on_changed(on_slider_change)
xmin_slider.on_changed(on_slider_change)
xmax_slider.on_changed(on_slider_change)
ymin_slider.on_changed(on_slider_change)
ymax_slider.on_changed(on_slider_change)
exposure_slider.on_changed(on_slider_change)

# Show the plot window
plt.show(block=True)
