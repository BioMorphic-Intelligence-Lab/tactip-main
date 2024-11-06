""" Script to live demo the sensor on a trained network.
"""
import os
import time
import matplotlib
matplotlib.use('TkAgg')  # Ensure a suitable backend is used

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
import matplotlib.patches as patches
import numpy as np

from tactile_image_processing.utils import load_json_obj
from tactile_image_processing.image_transforms import process_image
from tactile_learning.supervised.models import create_model
from tactile_data_shear.tactile_servo_control import BASE_MODEL_PATH
from tactile_image_processing.simple_sensors import RealSensor

from tactile_servo_control.utils.label_encoder import LabelEncoder
from tactile_servo_control.utils.labelled_model import LabelledModel
from tactile_servo_control.utils.parse_args import parse_args
from tactile_servo_control.collect_data.setup_collect_data import setup_sensor_image_params
from tactile_servo_control.collect_data.setup_collect_data import BBOX, CIRCLE_MASK_RADIUS, THRESH

MAX_PLOT_TIME = 10.0  # Maximum time to show on the plot (in seconds)
TIME_STEP = 0.1  # Time interval between frames (in seconds)

def setup_sensor(args):
    output_dir = '_'.join([args.robot, args.sensor])
    model_dir_name = '_'.join(filter(None, [args.models[0], *args.model_version]))
    
    model_dir = os.path.join(BASE_MODEL_PATH, output_dir, args.tasks[0], model_dir_name)
    model_params = load_json_obj(os.path.join(model_dir, 'model_params'))
    model_image_params = load_json_obj(os.path.join(model_dir, 'model_image_params'))
    model_label_params = load_json_obj(os.path.join(model_dir, 'model_label_params'))
    if os.path.isfile(os.path.join(model_dir, 'processed_image_params.json')):
        print("Using processed image params")
        sensor_image_params = load_json_obj(os.path.join(model_dir, 'processed_image_params'))
    else:
        print("Using sensor image params")
        sensor_image_params = load_json_obj(os.path.join(model_dir, 'sensor_image_params'))

    
    # setup the sensor
    embodiment = '_'.join([args.robot, args.sensor])
    processed_image_params = {
        "bbox": BBOX[embodiment],
        "circle_mask_radius": CIRCLE_MASK_RADIUS[embodiment],
        "thresh": THRESH[embodiment],
    }
    sensor_image_params = setup_sensor_image_params(robot='ur',sensor='tactip')
    sensor = RealSensor(sensor_image_params)
    
    # create the label encoder/decoder
    label_encoder = LabelEncoder(model_label_params, device=args.device)
    
    # setup the model
    model = create_model(
        in_dim=model_image_params['image_processing']['dims'],
        in_channels=1,
        out_dim=label_encoder.out_dim,
        model_params=model_params,
        saved_model_dir=model_dir,
        device=args.device
    )
    model.eval()

    pose_model = LabelledModel(
        model,
        model_image_params['image_processing'],
        label_encoder,
        device=args.device
    )
    
    return sensor, pose_model

def run(sensor, model):
    """Run the demo.
    """
    # setup the sensor
    embodiment = '_'.join([args.robot, args.sensor])
    processed_image_params = {
        "bbox": BBOX[embodiment],
        "circle_mask_radius": CIRCLE_MASK_RADIUS[embodiment],
        "thresh": THRESH[embodiment],
    }
    print(f"processed_image_params: {processed_image_params}")
    
    # Create the figure and subplots grid layout
    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(2, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1], wspace=0.3, hspace=0.3)

    # Top row left: Running plot
    ax1 = fig.add_subplot(gs[0, 0:2])
    ax1.set_title('Predicted pose components over time')
    ax2 = ax1.twinx()  # Create a second y-axis sharing the same x-axis
    ax1.set_xlabel('Time [s]')
    
    pose_z_data, pose_Rx_data, pose_Ry_data, time_data = [], [], [], []
    line1, = ax2.plot([], [], 'b-', label='Pose z (Right Y-Axis)')
    line2, = ax1.plot([], [], 'g-', label='Pose Rx (Left Y-Axis)')
    line3, = ax1.plot([], [], 'r-', label='Pose Ry (Left Y-Axis)')
    
    # Set up the left y-axis limits
    ax1.set_ylim(-25., 30.)
    ax1.set_ylabel("Sensor pose angles [deg]")
    ax1.legend(loc='lower left')

    # Set up the right y-axis limits (adjust based on your data)
    ax2.set_ylim(-1.0, 4.5)  
    ax2.set_ylabel('Sensor pose depth [mm]')
    ax2.legend(loc='lower right')

    # Top row right: 
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.axis('off')  # Hide axes as it's just for displaying text
    text_display = ax3.text(0.05, 0.5, '', fontsize=14, va='center')
        
    # Bottom row left: Image display 1
    ax4 = fig.add_subplot(gs[1, 0])
    image_data1 = np.random.rand(480, 640, 3)  # Initial random image data
    image_plot1 = ax4.imshow(image_data1, cmap='gray')
    ax4.set_title('1. Raw image')
    ax4.axis('off')

    # Bottom row middle: Image display 2
    ax5 = fig.add_subplot(gs[1, 1])
    image_data2 = np.random.rand(440, 440)  # Initial random image data
    image_plot2 = ax5.imshow(image_data2, cmap='gray')
    ax5.set_title('2. Sensor image')
    ax5.axis('off')
    
    # Bottom row right: Image display 3
    ax6 = fig.add_subplot(gs[1, 2])
    image_data3 = np.random.rand(440, 440)  # Initial random image data
    image_plot3 = ax6.imshow(image_data3, cmap='gray')
    ax6.set_title('3. Processed image')
    ax6.axis('off')    
    
    time_counter = 0.  # Initialize a time counter

    # Function to initialize the plot
    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        line3.set_data([], [])
        image_plot1.set_data(np.zeros((480, 640, 3)))
        image_plot2.set_data(np.zeros((440, 440)))
        image_plot3.set_data(np.zeros((440, 440)))
        text_display.set_text('')
        return line1, line2, line3, image_plot1, image_plot2, image_plot3, text_display

    # Function to update the plot at each frame
    def update(frame):
        x_value = frame
        print(f"frame: {frame}")
        nonlocal time_counter  # Use the external time_counter variable
        time_counter += TIME_STEP  # Increment time by a fixed interval (0.05 seconds here)
        
        start_time = time.time()
        # Update tactile image
        raw_img = sensor.read()
        print(f'Raw image shape and data type: {raw_img.shape}, {raw_img.dtype}')
        sensor_img = sensor.process()
        print(f'Sensor image shape and data type: {sensor_img.shape}, {sensor_img.dtype}')
        processed_img = process_image(sensor_img, **processed_image_params)
        print(f'Processed image shape and data type: {processed_img.shape}, {processed_img.dtype}')
        # predict pose from observations
        pred_pose = model.predict(processed_img)
        end_time = time.time()
        #print(f"frame: {frame}")
        print(f"\n Predicted pose: {pred_pose}")
        
        pose_z_data.append(pred_pose[2])
        pose_Rx_data.append(pred_pose[3])
        pose_Ry_data.append(pred_pose[4])
        time_data.append(time_counter)
        # Maintain only the last 10 seconds of data
        time_window_start = time_counter - MAX_PLOT_TIME
        while time_data and time_data[0] < time_window_start:
            time_data.pop(0)
            pose_z_data.pop(0)
            pose_Rx_data.pop(0)
            pose_Ry_data.pop(0)
        
        # trim data lists to the last 10 seconds
        time_window_start = max(0, time_counter - MAX_PLOT_TIME)
        pruned_time_data = [t for t in time_data if t >= time_window_start]
        pruned_z_data = pose_z_data[-len(pruned_time_data):]
        pruned_Rx_data = pose_Rx_data[-len(pruned_time_data):]
        pruned_Ry_data = pose_Ry_data[-len(pruned_time_data):]
        
        # Update the lines with the latest data (only last 10 seconds)
        line1.set_data(pruned_time_data, pruned_z_data)
        line2.set_data(pruned_time_data, pruned_Rx_data)
        line3.set_data(pruned_time_data, pruned_Ry_data)

        # Update x-axis to show only the last 10 seconds
        ax1.set_xlim(time_window_start, time_counter)
        ax2.set_xlim(time_window_start, time_counter)
        
        new_image_data1 = raw_img
        new_image_data2 = sensor_img.astype(np.float64)/255.0
        new_image_data3 = processed_img.astype(np.float64)/255.0
        image_plot1.set_data(raw_img)
        image_plot2.set_data(new_image_data2)
        image_plot3.set_data(new_image_data3)
        
        # Update numerical display with the last values
        text_display.set_text(f'Current Pose Values:\n'
                            f'Pose Z: {pred_pose[2]:.2f} [mm]\n'
                            f'Pose Rx: {pred_pose[3]:.2f} [deg]\n'
                            f'Pose Ry: {pred_pose[4]:.2f} [deg]\n'
                            f'Model evaluation time: {(end_time-start_time)*1000:.2f} [us]')

        return line1, line2, line3, image_plot1, image_plot2, image_plot3, text_display

    # Create the animation
    ani = animation.FuncAnimation(fig, update, frames=np.linspace(0, 10, 50), init_func=init, blit=False)

    # Show the combined plot with animation and image display
    plt.show()

if __name__ == "__main__":
    args = parse_args(
        robot='ur',
        sensor='tactip',
        tasks=['surface_3d'],
        models=['simple_cnn'],
        model_version=['replica'],
        run_version=[''],
        device='cuda'
    )
    sensor, model = setup_sensor(args)
    run(sensor, model)