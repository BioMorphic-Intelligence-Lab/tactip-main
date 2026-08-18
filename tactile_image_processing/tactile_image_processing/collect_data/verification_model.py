import os
import sys
import time
import cv2
import torch
import numpy as np
import pandas as pd
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

# Ecosystem / Framework Imports
from tactile_image_processing.utils import load_json_obj
from tactile_learning.supervised.models import create_model
from tactile_servo_control.utils.label_encoder import LabelEncoder


# ==========================================
# 1. PHIDGET REFERENCE FORCE SENSOR CLASS
# ==========================================
class ThreeAxisForceSensor:
    def __init__(self, serial_number=781122, ch_x=3, ch_y=2, ch_z=1):
        self.serial_number = serial_number
        self.ch_indices = {'x': ch_x, 'y': ch_y, 'z': ch_z}
        self.channels = {}
        self.slopes = {}
        self.offsets = {}
        
        loads = np.array([0, 10, 20, 30, 40, 50])
        raw_data = {
            'x': np.array([[0, 0.00001], [0.20403, 0.20365], [0.40791, 0.40758], [0.61135, 0.61062], [0.8158, 0.81428], [1.02052, 1.02052]]),
            'y': np.array([[0, 0.00001], [0.19724, 0.19826], [0.39500, 0.39555], [0.59232, 0.59236], [0.7896, 0.78961], [0.98688, 0.98688]]),
            'z': np.array([[0, 0.00001], [0.13881, 0.13873], [0.27818, 0.27779], [0.41715, 0.41653], [0.5564, 0.55581], [0.69500, 0.69500]])
        }
        
        for axis in ['x', 'y', 'z']:
            avg_mv_v = np.mean(raw_data[axis], axis=1)
            slope, offset = np.polyfit(avg_mv_v, loads, 1)
            self.slopes[axis] = slope
            self.offsets[axis] = offset

    def start(self):
        print("Connecting to PhidgetBridge processor...")
        for axis, ch_num in self.ch_indices.items():
            ch = VoltageRatioInput()
            ch.setDeviceSerialNumber(self.serial_number)
            ch.setChannel(ch_num)
            ch.openWaitForAttachment(5000)
            self.channels[axis] = ch
        print("Phidget Force Sensor Online.")

    def get_forces_in_newtons(self):
        mv_v_x = self.channels['x'].getVoltageRatio() * 1000.0
        mv_v_y = self.channels['y'].getVoltageRatio() * 1000.0
        mv_v_z = self.channels['z'].getVoltageRatio() * 1000.0
        
        fx = (mv_v_x * self.slopes['x']) + self.offsets['x']
        fy = (mv_v_y * self.slopes['y']) + self.offsets['y']
        fz = (mv_v_z * self.slopes['z']) + self.offsets['z']
        return fx, fy, fz

    def close(self):
        for ch in self.channels.values():
            ch.close()
        print("Phidget Hardware disconnected cleanly.")


# ==========================================
# 2. STANDALONE TACTIP PREPROCESSING ENGINE
# ==========================================
def process_tactip_frame_standalone(
    cv2_frame,
    bbox=(0, 0, 640, 480),          # Default crop bounding box (x0, y0, x1, y1)
    target_dims=(128, 128),
    thresh_params=[61, -50],
    circle_mask_radius=400
):
    """
    Explicit, standalone image processing pipeline matching TacTip dataset generation.
    Returns:
      norm_img: float32 numpy array [0.0, 1.0] of shape (128, 128)
      display_img: uint8 numpy array [0, 255] for HUD overlay display
    """
    # 1. Grayscale Conversion
    if len(cv2_frame.shape) == 3 and cv2_frame.shape[2] == 3:
        img = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2GRAY)
    else:
        img = cv2_frame.copy()

    # 2. Crop to Bounding Box (if specified)
    if bbox is not None:
        x0, y0, x1, y1 = bbox
        img = img[y0:y1, x0:x1]

    # 3. Adaptive Thresholding (Binary Pin-Marker Segmentation)
    if thresh_params is not None:
        block_size, c_val = thresh_params[0], thresh_params[1]
        img = cv2.adaptiveThreshold(
            img,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            c_val
        )

    # 4. Circular Masking (Mask out skin edge artifacts)
    if circle_mask_radius is not None:
        hh, ww = img.shape[:2]
        hc, wc = hh // 2, ww // 2
        mask = np.ones((hh, ww), dtype=np.uint8)
        cv2.circle(mask, (wc, hc), circle_mask_radius, 0, -1)
        img[mask == 1] = 0

    # 5. Resize to CNN target dimensions (128 x 128)
    if target_dims is not None:
        img = cv2.resize(img, tuple(target_dims), interpolation=cv2.INTER_AREA)

    # 6. Normalize float32 [0.0, 1.0]
    norm_img = img.astype(np.float32) / 255.0

    return norm_img, img


# ==========================================
# 3. MODEL LOADER AND INFERENCE ENGINE
# ==========================================
def load_tactip_model(model_dir="tactile_data_shear/models/ur_tactip/surface_9d/simple_cnn_A2_1", device="cuda"):
    """
    Loads model params from JSONs, builds model architecture, 
    and loads weights from best_model.pth.
    """
    device = torch.device(device if torch.cuda.is_available() else "cpu")
    
    model_params = load_json_obj(os.path.join(model_dir, "model_params"))
    label_params = load_json_obj(os.path.join(model_dir, "model_label_params"))
    model_image_params = load_json_obj(os.path.join(model_dir, "model_image_params"))
    
    label_encoder = LabelEncoder(label_params, device)
    
    model = create_model(
        in_dim=model_image_params['image_processing']['dims'],
        in_channels=1,
        out_dim=label_encoder.out_dim,
        model_params=model_params,
        device=device
    )
    
    model_path = os.path.join(model_dir, "best_model.pth")
    state_dict = torch.load(model_path, map_location=device, weights_only=True)
    model.load_state_dict(state_dict)
    
    model.to(device)
    model.eval()
    
    return model, label_encoder, model_image_params, device


def predict_tactip(model, label_encoder, cv2_frame, target_dims, device):
    """
    Preprocesses OpenCV camera frame and runs inference.
    """
    # Run our standalone mimicked processing routine
    norm_img, display_img = process_tactip_frame_standalone(
        cv2_frame,
        bbox=None,                    # Set to (x0, y0, x1, y1) if your embodiment uses camera cropping
        target_dims=target_dims,       # Usually (128, 128)
        thresh_params=[61, -50],       # TacTip standard thresholding
        circle_mask_radius=400        # TacTip circle mask
    )
    
    # Reshape (128, 128) -> Tensor (1, 1, 128, 128)
    tensor_img = torch.from_numpy(norm_img).unsqueeze(0).unsqueeze(0).to(device)
    
    with torch.no_grad():
        raw_pred = model(tensor_img)
        decoded_dict = label_encoder.decode_label(raw_pred)
        
    predictions = {
        label: float(val.item() if isinstance(val, torch.Tensor) else val)
        for label, val in decoded_dict.items()
    }
    
    return predictions, display_img


# ==========================================
# 4. MAIN REAL-TIME VERIFICATION LOOP
# ==========================================
def main():
    MODEL_DIR = "tactile_data_shear/models/ur_tactip/surface_9d/simple_cnn_A2_1"
    CAMERA_INDEX = 4
    TACTIP_ANGLE_DEG = 106.73  # Rotational offset between internal camera and physical tape mark
    
    print(f"Loading model framework from '{MODEL_DIR}'...")
    model, label_encoder, model_image_params, device = load_tactip_model(MODEL_DIR)
    target_dims = model_image_params['image_processing']['dims']
    print(f"Model loaded successfully on {device}.")

    # Initialize Hardware
    phidget = ThreeAxisForceSensor()
    phidget.start()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open TacTip camera at index {CAMERA_INDEX}")
        phidget.close()
        return

    # Software Zero-Tare Routine
    print("\n[TARE] Zeroing baseline offsets... Ensure TacTip is NOT touching anything.")
    time.sleep(1.0)
    
    phidget_samples_x, phidget_samples_y, phidget_samples_z = [], [], []
    tactip_samples_x, tactip_samples_y, tactip_samples_z = [], [], []

    for _ in range(15):
        ret, frame = cap.read()
        if ret:
            preds, _ = predict_tactip(model, label_encoder, frame, target_dims, device)
            tactip_samples_x.append(preds.get('Fx', 0.0))
            tactip_samples_y.append(preds.get('Fy', 0.0))
            tactip_samples_z.append(preds.get('Fz', 0.0))

        fx, fy, fz = phidget.get_forces_in_newtons()
        phidget_samples_x.append(fx)
        phidget_samples_y.append(fy)
        phidget_samples_z.append(fz)
        time.sleep(0.05)

    phidget_tare = (np.mean(phidget_samples_x), np.mean(phidget_samples_y), np.mean(phidget_samples_z))
    tactip_tare = (np.mean(tactip_samples_x), np.mean(tactip_samples_y), np.mean(tactip_samples_z))
    print("Zero-Tare calibration complete.")

    recording = False
    log_data = []
    
    print("\n--- Live Verification Streaming ---")
    print("  Press 'r' to START / STOP logging data to CSV")
    print("  Press 's' to PRINT MAE / RMSE performance report")
    print("  Press 'q' to QUIT\n")

    try:
        start_time = time.time()
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # 1. Get Reference Forces (Phidget)
            p_fx, p_fy, p_fz = phidget.get_forces_in_newtons()
            ref_fx = p_fx - phidget_tare[0]
            ref_fy = p_fy - phidget_tare[1]
            ref_fz = p_fz - phidget_tare[2]

            # 2. Get Model Predictions (TacTip)
            preds, proc_display = predict_tactip(model, label_encoder, frame, target_dims, device)
            
            raw_fx = preds.get('Fx', 0.0) - tactip_tare[0]
            raw_fy = preds.get('Fy', 0.0) - tactip_tare[1]
            pred_fz = preds.get('Fz', 0.0) - tactip_tare[2]

            # Apply 2D Rotation Matrix 
            theta_rad = np.radians(TACTIP_ANGLE_DEG)
            pred_fx = np.cos(theta_rad) * raw_fx - np.sin(theta_rad) * raw_fy
            pred_fy = np.sin(theta_rad) * raw_fx + np.cos(theta_rad) * raw_fy

            current_time = time.time() - start_time

            # 3. Log active data stream
            if recording:
                log_data.append({
                    'timestamp': current_time,
                    'ref_Fx': ref_fx, 'ref_Fy': ref_fy, 'ref_Fz': ref_fz,
                    'pred_Fx': pred_fx, 'pred_Fy': pred_fy, 'pred_Fz': pred_fz,
                    'error_Fx': abs(ref_fx - pred_fx),
                    'error_Fy': abs(ref_fy - pred_fy),
                    'error_Fz': abs(ref_fz - pred_fz)
                })

            # 4. Draw Telemetry HUD
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (460, 170), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

            cv2.putText(frame, "REAL-TIME FORCE VERIFICATION", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, "Axis | Ref (Phidget) | Model (TacTip) | Error", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
            cv2.putText(frame, "-"*55, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

            cv2.putText(frame, f"Fx:    {ref_fx:+7.3f} N    |    {pred_fx:+7.3f} N   | {abs(ref_fx-pred_fx):.3f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2)
            cv2.putText(frame, f"Fy:    {ref_fy:+7.3f} N    |    {pred_fy:+7.3f} N   | {abs(ref_fy-pred_fy):.3f}", (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2)
            cv2.putText(frame, f"Fz:    {ref_fz:+7.3f} N    |    {pred_fz:+7.3f} N   | {abs(ref_fz-pred_fz):.3f}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2)

            rec_str = f"[REC ACTIVE - {len(log_data)} pts]" if recording else "[PRESS 'R' TO RECORD]"
            rec_color = (0, 0, 255) if recording else (180, 180, 180)
            cv2.putText(frame, rec_str, (20, 158), cv2.FONT_HERSHEY_SIMPLEX, 0.45, rec_color, 2)

            # Draw thumbnail of preprocessed input in bottom right corner
            proc_bgr = cv2.cvtColor(proc_display, cv2.COLOR_GRAY2BGR)
            proc_resized = cv2.resize(proc_bgr, (120, 120))
            h, w, _ = frame.shape
            frame[h-130:h-10, w-130:w-10] = proc_resized
            cv2.rectangle(frame, (w-130, h-130), (w-10, h-10), (0, 255, 255), 1)
            cv2.putText(frame, "Model Input", (w-125, h-115), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

            cv2.imshow("TacTip Real-Time Verification Suite", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                recording = not recording
                status = "STARTED" if recording else "STOPPED"
                print(f"\n[LOGGING {status}] Current samples collected: {len(log_data)}")
            elif key == ord('s'):
                if len(log_data) > 0:
                    df_log = pd.DataFrame(log_data)
                    print("\n" + "="*50)
                    print("--- LIVE VERIFICATION PERFORMANCE SUMMARY ---")
                    print(f"Total Samples Recorded: {len(df_log)}")
                    print(f"Fx MAE: {df_log['error_Fx'].mean():.4f} N | RMSE: {np.sqrt((df_log['error_Fx']**2).mean()):.4f} N")
                    print(f"Fy MAE: {df_log['error_Fy'].mean():.4f} N | RMSE: {np.sqrt((df_log['error_Fy']**2).mean()):.4f} N")
                    print(f"Fz MAE: {df_log['error_Fz'].mean():.4f} N | RMSE: {np.sqrt((df_log['error_Fz']**2).mean()):.4f} N")
                    print("="*50 + "\n")
                else:
                    print("\nNo logged data yet. Press 'r' to log data while probing the sensor.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        phidget.close()

        if len(log_data) > 0:
            df_log = pd.DataFrame(log_data)
            output_csv = "physical_verification_results.csv"
            df_log.to_csv(output_csv, index=False)
            print(f"\nSaved {len(df_log)} logged verification records to '{output_csv}'.")

if __name__ == "__main__":
    main()