""" Script to live demo the sensor on a trained network.
"""
import os
import time
import matplotlib
matplotlib.use('TkAgg')  # Ensure a suitable backend is used

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from skimage.metrics import structural_similarity as ssim

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
        print(f"Using processed image params from {os.path.join(model_dir, 'processed_image_params.json')}")
        sensor_image_params = load_json_obj(os.path.join(model_dir, 'processed_image_params'))
    else:
        print("Using sensor image params")
        sensor_image_params = load_json_obj(os.path.join(model_dir, 'sensor_image_params'))

    
    # setup the sensor
    embodiment = '_'.join([args.robot, args.sensor])
    # processed_image_params = {
    #     "bbox": BBOX[embodiment],
    #     "circle_mask_radius": CIRCLE_MASK_RADIUS[embodiment],
    #     "thresh": THRESH[embodiment],
    # }
    # # sensor_image_params = setup_sensor_image_params(robot='ur',sensor='aerial-A3')
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
    print(f"label names: {pose_model.target_label_names}")
    return sensor, pose_model

def run(sensor, model):
    """Run the demo.
    """
    # setup the sensor
    embodiment = '_'.join([args.robot, args.sensor])
    # processed_image_params = {
    #     "bbox": BBOX[embodiment],
    #     "circle_mask_radius": CIRCLE_MASK_RADIUS[embodiment],
    #     "thresh": THRESH[embodiment],
    # }
    # print(f"processed_image_params: {processed_image_params}")

    # Capture ref image for SSIM
    processed_image_params = {
        "bbox": sensor.sensor_params.get("bbox", [500,500]),
        "circle_mask_radius": sensor.sensor_params.get("circle_mask_radius", 500),
        "thresh": sensor.sensor_params.get("thresh", [10,-10]),
    }
    ref_img_sensor = sensor.process()
    #ref_img_processed = process_image(ref_img_sensor, **processed_image_params)

    # Create the figure and subplots grid layout
    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(2, 4, width_ratios=[1, 1, 1, 1], height_ratios=[1, 1], wspace=0.3, hspace=0.3)

    # Top row left: Running plot
    ax1 = fig.add_subplot(gs[0, 0:3])
    if 'shear_x' in model.target_label_names:
        ax1.set_title('Predicted pose- and shear components over time')
    else:
        ax1.set_title('Predicted pose components over time')
    ax2 = ax1.twinx()  # Create a second y-axis sharing the same x-axis
    ax1.set_xlabel('Time [s]')
    print(model.target_label_names)
    
    time_data = []
    pose_data = {label: [] for label in model.target_label_names}
    
    line_list = []
    for label_name in model.target_label_names:
        if "R" in label_name: # Check if label is rotation -->ax1
            line, = ax1.plot([], [], label=f'{label_name} (Left Y-axis)')
        else: # Otherwise it's a translation --> ax2
            line, = ax2.plot([], [],c='tab:green', label=f'{label_name} (Right Y-axis)')
        line_list.append(line)
    
    # Set up the left y-axis limits
    ax1.set_ylim(-25., 30.)
    ax1.set_ylabel("Sensor pose angles [deg]")
    ax1.legend(loc='lower left')

    # Set up the right y-axis limits (adjust based on your data)
    ax2.set_ylim(-4.5, 1.5)  
    ax2.set_ylabel('Sensor pose depth [mm]')
    ax2.legend(loc='lower right')

    # Top row right: 
    ax3 = fig.add_subplot(gs[0, 3])
    ax3.axis('off')  # Hide axes as it's just for displaying text
    text_display = ax3.text(0.05, 0.5, '', fontsize=14, va='center')
    
    # Bottom row far left: Ref image for SSIM
    ax7 = fig.add_subplot(gs[1, 0])
    image_plot_ref = ax7.imshow(ref_img_sensor, cmap='gray')
    ax7.set_title('SSIM ref image')
    ax7.axis('off')

    # Bottom row left: Image display 1
    ax4 = fig.add_subplot(gs[1, 1])
    image_data1 = np.random.rand(480, 640, 3)  # Initial random image data
    image_plot1 = ax4.imshow(image_data1, cmap='gray')
    ax4.set_title('1. Raw image')
    ax4.axis('off')

    # Bottom row middle: Image display 2
    ax5 = fig.add_subplot(gs[1, 2])
    image_data2 = np.random.rand(440, 440)  # Initial random image data
    image_plot2 = ax5.imshow(image_data2, cmap='gray')
    ax5.set_title('2. Sensor image')
    ax5.axis('off')
    
    # Bottom row right: Image display 3
    ax6 = fig.add_subplot(gs[1, 3])
    image_data3 = np.random.rand(440, 440)  # Initial random image data
    image_plot3 = ax6.imshow(image_data3, cmap='gray')
    ax6.set_title('3. Processed image')
    ax6.axis('off')    
    
    time_counter = 0.  # Initialize a time counter

    # Function to initialize the plot
    def init():
        for line in line_list:
            line.set_data([], [])
        image_plot_ref.set_data(ref_img_sensor)
        image_plot1.set_data(np.zeros((480, 640, 3)))
        image_plot2.set_data(np.zeros((440, 440)))
        image_plot3.set_data(np.zeros((440, 440)))
        text_display.set_text('')
        return line_list + [image_plot_ref, image_plot1, image_plot2, image_plot3, text_display]

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
        #processed_img = process_image(sensor_img, **processed_image_params)
        #print(f'Processed image shape and data type: {processed_img.shape}, {processed_img.dtype}')
        
        ssim_score_sensor, dif_sensor = ssim(ref_img_sensor.squeeze(), sensor_img.squeeze(), full=True)
        # ssim_score_processed, dif_proc = ssim(ref_img_processed.squeeze(), processed_img.squeeze(), full=True)
        
        # predict pose from observations
        pred_pose = model.predict(sensor_img) # Output is np.array of dim 1 and size 12
        end_time = time.time()
        #print(f"frame: {frame}")
        # Rotation between output in actual frame and the end-effector frame

        rot_pred_pos = np.matmul(np.array([[1, 0, 0], [0, -1, 0],[0, 0, -1]]), pred_pose[:3])
        rot_pred_ang = np.matmul(np.array([[1, 0, 0], [0, -1, 0],[0, 0, -1]]), pred_pose[3:6])
        rot_pred_pose = np.concatenate([rot_pred_pos, rot_pred_ang])
        print(f"\n Predicted pose: {rot_pred_pose}")
        
        # Append new data
        time_data.append(time_counter)
        for i, label in enumerate(model.label_names):
            if label in pose_data.keys():
                pose_data[label].append(rot_pred_pose[i])

        # Maintain only the last 10 seconds of data
        time_window_start = time_counter - MAX_PLOT_TIME
        while time_data and time_data[0] < time_window_start:
            time_data.pop(0)
            for label in model.target_label_names:
                pose_data[label].pop(0)
        
        # trim data lists to the last 10 seconds
        time_window_start = max(0, time_counter - MAX_PLOT_TIME)
        
        # Update lines
        pruned_time_data = [t for t in time_data if t >= time_window_start]
        for i, label in enumerate(model.target_label_names):
            pruned_data = pose_data[label][-len(pruned_time_data):]
            line_list[i].set_data(pruned_time_data, pruned_data)

        # Update x-axis to show only the last 10 seconds
        ax1.set_xlim(time_window_start, time_counter)
        ax2.set_xlim(time_window_start, time_counter)
        
        new_image_data1 = raw_img
        new_image_data2 = sensor_img.astype(np.float64)/255.0
        new_image_data3 = sensor_img.astype(np.float64)/255.0 # can replace for processed image if needed
        image_plot1.set_data(new_image_data1)
        image_plot2.set_data(new_image_data2)
        image_plot3.set_data(new_image_data3)
        
        # Update numerical display
        text_display.set_text(
            '\n'.join([f'{label}: {pose_data[label][-1]:.2f}' for i, label in enumerate(model.target_label_names)]) +
            f'\nModel evaluation time: {(end_time - start_time) * 1000:.2f} [ms]' +
            f'\nFPS: {1.0 / (end_time - start_time):.2f}' +
            f'\nSSIM score sensor: {ssim_score_sensor:.2f}'
#           + f'\nSSIM score processed: {ssim_score_processed:.2f}'
        )

        return line_list + [image_plot1, image_plot2, image_plot3, text_display]

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
        model_version=['A3_2025'],
        run_version=[''],
        device='cuda'
    )
    sensor, model = setup_sensor(args)
    run(sensor, model)