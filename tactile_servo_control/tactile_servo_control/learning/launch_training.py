"""
python launch_training.py -r sim -s tactip -m simple_cnn -t edge_2d
"""
import os
import itertools as it
import torch

from tactile_data_shear.tactile_servo_control_paths import BASE_DATA_PATH, BASE_MODEL_PATH
from tactile_image_processing.utils import make_dir
from tactile_learning.supervised.image_generator import ImageDataGenerator
from tactile_learning.supervised.models import create_model
from tactile_learning.supervised.train_model import train_model
from tactile_learning.utils.utils_learning import seed_everything
from tactile_learning.utils.utils_plots import RegressionPlotter

from tactile_servo_control.learning.setup_training import setup_training, csv_row_to_label
from tactile_servo_control.prediction.evaluate_model import evaluate_model
from tactile_servo_control.utils.label_encoder import LabelEncoder
from tactile_servo_control.utils.parse_args import parse_args


def launch(args):

    output_dir = '_'.join([args.robot, args.sensor])

    for args.data_dim, args.output_dim, args.model in it.product(args.data_dims, args.output_dims, args.models):

        model_dir_name = '_'.join(filter(None, [args.model, *args.model_version]))

        # data dirs - list of directories combined in generator
        train_data_dirs = [
            os.path.join(BASE_DATA_PATH, output_dir, args.data_dim, d) for d in args.train_dirs
        ]
        val_data_dirs = [
            os.path.join(BASE_DATA_PATH, output_dir, args.data_dim, d) for d in args.val_dirs
        ]

        # setup save dir
        save_dir = os.path.join(BASE_MODEL_PATH, output_dir, args.output_dim, model_dir_name)
        make_dir(save_dir)
        
        # give feedback to user
        print(f"Getting training data from {train_data_dirs}")
        print(f"Getting validation data from {val_data_dirs}")
        print(f"Saving model to {save_dir}")

        # setup parameters
        learning_params, model_params, label_params, image_params = setup_training(
            args.model,
            args.output_dim,
            train_data_dirs,
            save_dir
        )

        # configure dataloaders
        train_generator = ImageDataGenerator(
            train_data_dirs,
            csv_row_to_label,
            **{**image_params['image_processing'], **image_params['augmentation']}
        )
        val_generator = ImageDataGenerator(
            val_data_dirs,
            csv_row_to_label,
            **image_params['image_processing']
        )
        
        # give user feedback about loaded data
        print(f"Loaded {len(train_generator)} training samples")
        print(f"Loaded {len(val_generator)} validation samples")
        print(f"Training for {learning_params['epochs']} epochs")
        print(f"Batch size: {learning_params['batch_size']}")
        print(f"Learning rate: {learning_params['lr']}")
        print(f"Model: {args.model}")
        print(f"Data dimension: {args.data_dim}")
        print(f"Device: {args.device}")

        # create the label encoder/decoder and plotter
        label_encoder = LabelEncoder(label_params, args.device)
        error_plotter = RegressionPlotter(label_params, save_dir, final_only=False)

        # create the model
        seed_everything(learning_params['seed'])
        model = create_model(
            in_dim=image_params['image_processing']['dims'],
            in_channels=1,
            out_dim=label_encoder.out_dim,
            model_params=model_params,
            device=args.device
        )

        train_model(
            prediction_mode='regression',
            model=model,
            label_encoder=label_encoder,
            train_generator=train_generator,
            val_generator=val_generator,
            learning_params=learning_params,
            save_dir=save_dir,
            error_plotter=error_plotter,
            device=args.device
        )

        # perform a final evaluation using the last model
        evaluate_model(
            model,
            label_encoder,
            val_generator,
            learning_params,
            error_plotter,
            device=args.device
        )


if __name__ == "__main__":

    args = parse_args(
        robot='ur',
        sensor='aerial-B1',
        data_dims=['surface_9d'],
        output_dims = ['surface_5d'],
        train_dirs=['train_data'],
        val_dirs=['val_data'],
        models=['simple_cnn'],
        model_version=['B1_2026'],
        device='cuda'
    )
    
    torch.cuda.empty_cache()

    launch(args)
