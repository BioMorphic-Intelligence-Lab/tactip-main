import time
import sys
import numpy as np
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

class ThreeAxisForceSensor:
    def __init__(self, serial_number=781122, ch_x=3, ch_y=2, ch_z=1):
        self.serial_number = serial_number
        self.ch_indices = {'x': ch_x, 'y': ch_y, 'z': ch_z}
        self.channels = {}
        
        self.slopes = {}
        self.offsets = {}
        
        loads = np.array([0, 10, 20, 30, 40, 50])
        
        raw_data = {
            'x': np.array([
                [0, 0.00001],
                [0.20403, 0.20365],
                [0.40791, 0.40758],
                [0.61135, 0.61062],
                [0.8158, 0.81428],
                [1.02052, 1.02052]
            ]),
            'y': np.array([
                [0, 0.00001],
                [0.19724, 0.19826],
                [0.39500, 0.39555],
                [0.59232, 0.59236],
                [0.7896, 0.78961],
                [0.98688, 0.98688]
            ]),
            'z': np.array([
                [0, 0.00001],
                [0.13881, 0.13873],
                [0.27818, 0.27779],
                [0.41715, 0.41653],
                [0.5564, 0.55581],
                [0.69500, 0.69500]
            ])
        }
        
        # Compute calibration factors dynamically using averaged hysteresis loops
        for axis in ['x', 'y', 'z']:
            # Average the Process and Return Trip paths to handle sensor hysteresis
            avg_mv_v = np.mean(raw_data[axis], axis=1)
            
            # Linear regression: Force (N) = Slope * (mV/V) + Offset
            slope, offset = np.polyfit(avg_mv_v, loads, 1)
            
            self.slopes[axis] = slope
            self.offsets[axis] = offset

    def start(self):
        """Initializes and establishes a stable connection to all Phidget channels."""
        print("Connecting to PhidgetBridge processor...")
        for axis, ch_num in self.ch_indices.items():
            ch = VoltageRatioInput()
            ch.setDeviceSerialNumber(self.serial_number)
            ch.setChannel(ch_num)
            ch.openWaitForAttachment(5000)
            self.channels[axis] = ch
        print(f"Force sensor online. Active channels -> Fx: Ch{self.ch_indices['x']}, Fy: Ch{self.ch_indices['y']}, Fz: Ch{self.ch_indices['z']}")

    def get_forces_in_newtons(self):
        """
        Polls the raw V/V signals from the processor, standardizes to mV/V,
        and applies the dynamic calibration parameters to calculate forces.
        """
        # Read current raw voltage ratio (V/V) and convert to mV/V (* 1000)
        mv_v_x = self.channels['x'].getVoltageRatio() * 1000.0
        mv_v_y = self.channels['y'].getVoltageRatio() * 1000.0
        mv_v_z = self.channels['z'].getVoltageRatio() * 1000.0
        
        # Convert to Newtons: Force = (mV/V * k) + b
        fx = (mv_v_x * self.slopes['x']) + self.offsets['x']
        fy = (mv_v_y * self.slopes['y']) + self.offsets['y']
        fz = (mv_v_z * self.slopes['z']) + self.offsets['z']
        
        return fx, fy, fz

    def close(self):
        """Safely terminates all open hardware attachments."""
        for ch in self.channels.values():
            ch.close()
        print("\nHardware disconnected cleanly.")



if __name__ == "__main__":
    sensor = ThreeAxisForceSensor()
    
    try:
        sensor.start()
        
        # Automated Software Zero-Tare Routine
        print("\nTaring sensor baseline offsets... Do not touch the sensor structure.")
        time.sleep(1.5)  # Let signals stabilize
        
        # Take 10 rapid samples to build a steady baseline average
        samples_x, samples_y, samples_z = [], [], []
        for _ in range(10):
            fx, fy, fz = sensor.get_forces_in_newtons()
            samples_x.append(fx)
            samples_y.append(fy)
            samples_z.append(fz)
            time.sleep(0.05)
            
        tare_x = np.mean(samples_x)
        tare_y = np.mean(samples_y)
        tare_z = np.mean(samples_z)
        print("Tare calibration successful.")
        print(f"Subtracted Baselines -> Fx: {tare_x:.4f} N | Fy: {tare_y:.4f} N | Fz: {tare_z:.4f} N\n")
        
        print("Streaming net forces in Newtons. Press Ctrl+C to terminate test.\n")
        print("       Fx (N)       |       Fy (N)       |       Fz (N)       ")
        print("-" * 70)
        
        # Live Force Monitoring Stream
        while True:
            raw_fx, raw_fy, raw_fz = sensor.get_forces_in_newtons()
            
            # Apply zero-tare offsets
            net_fx = raw_fx - tare_x
            net_fy = raw_fy - tare_y
            net_fz = raw_fz - tare_z
            
            # Print live updates 
            sys.stdout.write(f"\r   {net_fx:+12.4f}     |    {net_fy:+12.4f}     |    {net_fz:+12.4f}   ")
            sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz readout
            
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()