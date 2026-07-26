#!/usr/bin/env python3
import sys
import os
import time
import threading
import yaml
import tkinter as tk
from tkinter import ttk, messagebox
import rclpy
from std_msgs.msg import Float32MultiArray

class MotorMapperGUI:
    def __init__(self, root, node, pwm_pub):
        self.root = root
        self.node = node
        self.pwm_pub = pwm_pub
        
        self.root.title("PlaDyPos USV Motor Mapper")
        self.root.geometry("600x550")
        self.root.configure(bg="#1a1a1e")
        
        # Style
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure(".", background="#1a1a1e", foreground="#ffffff")
        self.style.configure("TLabel", background="#1a1a1e", foreground="#ffffff", font=("Helvetica", 11))
        self.style.configure("TButton", background="#2a2a30", foreground="#ffffff", borderwidth=1, font=("Helvetica", 10, "bold"))
        self.style.map("TButton", background=[("active", "#00adb5")], foreground=[("active", "#ffffff")])
        
        # State
        self.active_channel = None
        self.stop_timer = None
        self.selections = {1: tk.StringVar(value="Select..."),
                           2: tk.StringVar(value="Select..."),
                           3: tk.StringVar(value="Select..."),
                           4: tk.StringVar(value="Select...")}
        
        self.positions = ["Front Left (FL)", "Front Right (FR)", "Back Left (BL)", "Back Right (BR)"]
        
        # Create Layout
        self.create_widgets()
        
    def create_widgets(self):
        # Header
        header = tk.Label(self.root, text="USV Motor Mapping Diagnostics", bg="#1a1a1e", fg="#00adb5", font=("Helvetica", 16, "bold"))
        header.pack(pady=15)
        
        # Instructions
        inst = tk.Label(self.root, text="Click a channel button to spin the motor at 30% thrust.\nObserve which motor spins, and select its physical location.", 
                        bg="#1a1a1e", fg="#aaaaaa", font=("Helvetica", 10))
        inst.pack(pady=5)
        
        # Channel frames
        main_frame = tk.Frame(self.root, bg="#1a1a1e")
        main_frame.pack(pady=20)
        
        self.buttons = {}
        for ch in range(1, 5):
            ch_frame = tk.Frame(main_frame, bg="#25252b", bd=2, relief="groove", padx=10, pady=10)
            ch_frame.grid(row=(ch-1)//2, column=(ch-1)%2, padx=15, pady=15)
            
            lbl = tk.Label(ch_frame, text=f"Channel {ch}", bg="#25252b", fg="#ffffff", font=("Helvetica", 12, "bold"))
            lbl.pack(pady=5)
            
            btn = tk.Button(ch_frame, text=f"Test Channel {ch}", bg="#3a3a45", fg="#ffffff", activebackground="#00adb5", activeforeground="#ffffff",
                            font=("Helvetica", 10, "bold"), width=15, command=lambda c=ch: self.toggle_channel(c))
            btn.pack(pady=5)
            self.buttons[ch] = btn
            
            # Dropdown selection
            cb = ttk.Combobox(ch_frame, textvariable=self.selections[ch], values=self.positions, state="readonly", width=18)
            cb.pack(pady=5)
            
        # Global Control Frame
        ctrl_frame = tk.Frame(self.root, bg="#1a1a1e")
        ctrl_frame.pack(pady=20)
        
        stop_all_btn = tk.Button(ctrl_frame, text="STOP ALL MOTORS", bg="#d9534f", fg="#ffffff", activebackground="#c9302c", activeforeground="#ffffff",
                                 font=("Helvetica", 11, "bold"), width=20, command=self.stop_all)
        stop_all_btn.pack(side=tk.LEFT, padx=10)
        
        save_btn = tk.Button(ctrl_frame, text="Save Mapping", bg="#5cb85c", fg="#ffffff", activebackground="#4cae4c", activeforeground="#ffffff",
                             font=("Helvetica", 11, "bold"), width=15, command=self.save_mapping)
        save_btn.pack(side=tk.LEFT, padx=10)
        
        # Status Label
        self.status_lbl = tk.Label(self.root, text="Status: Ready", bg="#1a1a1e", fg="#00adb5", font=("Helvetica", 10, "italic"))
        self.status_lbl.pack(pady=10)

    def toggle_channel(self, ch):
        if self.active_channel == ch:
            self.stop_all()
        else:
            self.stop_all()
            self.active_channel = ch
            self.buttons[ch].configure(text="STOP", bg="#d9534f")
            self.status_lbl.configure(text=f"Status: Spinning Channel {ch}...", fg="#5cb85c")
            self.send_thrust(ch, 0.3)
            
            # Auto stop after 4 seconds for safety
            self.stop_timer = self.root.after(4000, self.stop_all)

    def send_thrust(self, ch, thrust):
        msg = Float32MultiArray()
        data = [0.0, 0.0, 0.0, 0.0]
        if ch is not None:
            data[ch-1] = float(thrust)
        msg.data = data
        self.pwm_pub.publish(msg)

    def stop_all(self):
        if self.stop_timer is not None:
            self.root.after_cancel(self.stop_timer)
            self.stop_timer = None
            
        if self.active_channel is not None:
            ch = self.active_channel
            self.buttons[ch].configure(text=f"Test Channel {ch}", bg="#3a3a45")
            self.active_channel = None
            
        self.send_thrust(None, 0.0)
        self.status_lbl.configure(text="Status: Stopped", fg="#d9534f")

    def save_mapping(self):
        # Validate that all channels have a unique selection
        mapping = {}
        selected_positions = []
        for ch in range(1, 5):
            val = self.selections[ch].get()
            if val == "Select...":
                messagebox.showerror("Error", f"Please select a physical location for Channel {ch}!")
                return
            mapping[ch] = val
            selected_positions.append(val)
            
        if len(set(selected_positions)) < 4:
            messagebox.showerror("Error", "Duplicate motor positions selected! Each channel must map to a unique position.")
            return
            
        # Success! Output mapping
        output_str = "--- MOTOR MAPPING RESULTS ---\n"
        for ch in range(1, 5):
            output_str += f"Channel {ch} -> {mapping[ch]}\n"
        
        # Save to file
        yaml_path = '/home/brayan/ros2_ws/src/pladypos/config/motor_mapping.yaml'
        try:
            with open(yaml_path, 'w') as f:
                yaml.dump({"motor_mapping": mapping}, f)
            output_str += f"\nSaved mapping to: {yaml_path}\n"
        except Exception as e:
            output_str += f"\nWarning: Could not save to config file: {e}\n"
            
        messagebox.showinfo("Mapping Saved", output_str)
        print(output_str)

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    
    # Load namespace from config
    usv_id = 5
    try:
        config_path = '/home/brayan/ros2_ws/src/pladypos/config/usv_config.yaml'
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                usv_id = config_data.get('usv_id', 5)
    except Exception:
        pass
    ns = f"usv{usv_id}"
    
    node = rclpy.create_node('motor_mapper')
    pwm_pub = node.create_publisher(Float32MultiArray, f'/{ns}/pwm_out', 10)
    
    # Start ROS spin thread
    thr = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thr.start()
    
    # Start Tkinter GUI
    root = tk.Tk()
    app = MotorMapperGUI(root, node, pwm_pub)
    
    def on_closing():
        app.stop_all()
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()
