#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
🚁 Quadcopter GUI Teleoperation System (English Version)
Control drones with an intuitive graphical interface.

Key Features:
- Graphical control panel
- Real-time status monitoring
- Joystick-style control
- Integrated safety features
- Mission planning tools
- Real-time map display
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from std_msgs.msg import Bool, String, Float32
import tkinter as tk
from tkinter import ttk, messagebox, Canvas
import threading
import time
import math
import sys
import signal
from collections import defaultdict
import numpy as np

class DroneGUIController(Node):
    def __init__(self):
        super().__init__('gui_teleop_controller')
        
        # ROS2 Publishers
        self.target_publisher = self.create_publisher(Point, '/ppo_target', 10)
        self.enable_publisher = self.create_publisher(Bool, '/ppo_enable', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/drone_status', 10)
        
        # ROS2 Subscribers (if needed)
        # self.position_subscriber = self.create_subscription(Point, '/drone_position', self.position_callback, 10)
        
        # Control state
        self.control_enabled = False
        self.current_position = [0.0, 0.0, 2.0]
        self.target_position = [0.0, 0.0, 2.0]
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.speed_level = 3  # 1-5 speed levels
        self.mission_active = False
        self.mission_type = None
        
        # Safety limits
        self.max_distance = 20.0  # Maximum distance from origin
        self.max_altitude = 8.0   # Maximum altitude
        self.min_altitude = 0.5   # Minimum altitude
        
        # Mission parameters
        self.mission_points = []
        self.current_mission_point = 0
        
        # GUI setup
        self.setup_gui()
        
        # Key bindings
        self.setup_key_bindings()
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        # GUI update timer
        self.gui_update_timer = self.create_timer(0.1, self.update_gui)  # 10Hz
        
        # Mission timer
        self.mission_timer = None
        
    def setup_gui(self):
        """Setup the GUI interface"""
        self.root = tk.Tk()
        self.root.title("Drone GUI Teleoperation System")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2c3e50')
        
        # Configure style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        
        # Main frame
        main_frame = tk.Frame(self.root, bg='#2c3e50')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left panel - Controls
        self.setup_control_panel(main_frame)
        
        # Center panel - Map
        self.setup_map_panel(main_frame)
        
        # Right panel - Status
        self.setup_status_panel(main_frame)
        
        # Bottom panel - Mission controls
        self.setup_mission_panel(main_frame)
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def setup_control_panel(self, parent):
        """Setup control panel"""
        control_frame = tk.LabelFrame(parent, text="Control Panel", 
                                    font=("Arial", 12, "bold"),
                                    bg='#34495e', fg='white', bd=2)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Basic controls
        basic_frame = tk.Frame(control_frame, bg='#34495e')
        basic_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Enable/Disable control
        self.control_btn = tk.Button(basic_frame, text="Enable Control", 
                                   command=self.toggle_control,
                                   bg='#27ae60', fg='white', font=("Arial", 10, "bold"),
                                   width=15, height=2)
        self.control_btn.pack(pady=5)
        
        # Takeoff/Landing
        self.takeoff_btn = tk.Button(basic_frame, text="Takeoff", 
                                   command=self.takeoff,
                                   bg='#3498db', fg='white', font=("Arial", 10, "bold"),
                                   width=15, height=2)
        self.takeoff_btn.pack(pady=2)
        
        self.land_btn = tk.Button(basic_frame, text="Land", 
                                command=self.land,
                                bg='#e67e22', fg='white', font=("Arial", 10, "bold"),
                                width=15, height=2)
        self.land_btn.pack(pady=2)
        
        # Return to home
        self.home_btn = tk.Button(basic_frame, text="Return Home", 
                                command=self.return_home,
                                bg='#9b59b6', fg='white', font=("Arial", 10, "bold"),
                                width=15, height=2)
        self.home_btn.pack(pady=2)
        
        # Emergency landing
        self.emergency_btn = tk.Button(basic_frame, text="EMERGENCY LAND", 
                                     command=self.emergency_land,
                                     bg='#e74c3c', fg='white', font=("Arial", 10, "bold"),
                                     width=15, height=2)
        self.emergency_btn.pack(pady=5)
        
        # Speed control
        speed_frame = tk.Frame(control_frame, bg='#34495e')
        speed_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Label(speed_frame, text="Speed Level", 
               bg='#34495e', fg='white', font=("Arial", 10, "bold")).pack()
        
        self.speed_var = tk.IntVar(value=3)
        self.speed_scale = tk.Scale(speed_frame, from_=1, to=5, orient=tk.HORIZONTAL,
                                  variable=self.speed_var, command=self.update_speed,
                                  bg='#34495e', fg='white', font=("Arial", 9),
                                  length=200)
        self.speed_scale.pack(pady=5)
        
        # Direction control pad
        self.setup_direction_pad(control_frame)
        
    def setup_direction_pad(self, parent):
        """Setup direction control pad"""
        dir_frame = tk.LabelFrame(parent, text="Direction Control", 
                                font=("Arial", 11, "bold"),
                                bg='#34495e', fg='white')
        dir_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Direction buttons grid
        button_frame = tk.Frame(dir_frame, bg='#34495e')
        button_frame.pack(pady=10)
        
        # Forward/Backward
        tk.Button(button_frame, text="↑\nForward", command=lambda: self.move_direction('forward'),
                bg='#2980b9', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=0, column=1, padx=2, pady=2)
        
        tk.Button(button_frame, text="↓\nBackward", command=lambda: self.move_direction('backward'),
                bg='#2980b9', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=2, column=1, padx=2, pady=2)
        
        # Left/Right
        tk.Button(button_frame, text="←\nLeft", command=lambda: self.move_direction('left'),
                bg='#2980b9', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=1, column=0, padx=2, pady=2)
        
        tk.Button(button_frame, text="→\nRight", command=lambda: self.move_direction('right'),
                bg='#2980b9', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=1, column=2, padx=2, pady=2)
        
        # Up/Down
        tk.Button(button_frame, text="▲\nUp", command=lambda: self.move_direction('up'),
                bg='#27ae60', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=0, column=3, padx=2, pady=2)
        
        tk.Button(button_frame, text="▼\nDown", command=lambda: self.move_direction('down'),
                bg='#27ae60', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=2, column=3, padx=2, pady=2)
        
        # Rotation
        tk.Button(button_frame, text="↺\nLeft Turn", command=lambda: self.move_direction('turn_left'),
                bg='#8e44ad', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=0, column=0, padx=2, pady=2)
        
        tk.Button(button_frame, text="↻\nRight Turn", command=lambda: self.move_direction('turn_right'),
                bg='#8e44ad', fg='white', font=("Arial", 9, "bold"),
                width=8, height=2).grid(row=0, column=2, padx=2, pady=2)
        
    def setup_map_panel(self, parent):
        """Setup map display panel"""
        map_frame = tk.LabelFrame(parent, text="Drone Arena Map", 
                                font=("Arial", 12, "bold"),
                                bg='#34495e', fg='white', bd=2)
        map_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        # Configure grid weights
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=1)
        
        # Map canvas
        self.map_canvas = Canvas(map_frame, bg='white', width=500, height=500)
        self.map_canvas.pack(padx=10, pady=10)
        
        # Bind mouse click for target setting
        self.map_canvas.bind("<Button-1>", self.on_map_click)
        
        # Initialize map
        self.draw_map()
        
    def setup_status_panel(self, parent):
        """Setup status display panel"""
        status_frame = tk.LabelFrame(parent, text="Status Monitor", 
                                   font=("Arial", 12, "bold"),
                                   bg='#34495e', fg='white', bd=2)
        status_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        
        # Status labels
        self.status_labels = {}
        
        labels = [
            ("Position", "current_pos"),
            ("Target", "target_pos"),
            ("Altitude", "altitude"),
            ("Yaw", "yaw"),
            ("Speed Level", "speed"),
            ("Control", "control"),
            ("Mission", "mission"),
            ("Distance to Home", "dist_home"),
            ("Distance to Target", "dist_target"),
            ("Safety Status", "safety")
        ]
        
        for i, (label, key) in enumerate(labels):
            tk.Label(status_frame, text=f"{label}:", 
                   bg='#34495e', fg='#ecf0f1', font=("Arial", 10, "bold")).grid(
                   row=i, column=0, sticky="w", padx=10, pady=3)
            
            self.status_labels[key] = tk.Label(status_frame, text="--", 
                                             bg='#34495e', fg='#3498db', 
                                             font=("Arial", 10))
            self.status_labels[key].grid(row=i, column=1, sticky="w", padx=10, pady=3)
        
    def setup_mission_panel(self, parent):
        """Setup mission control panel"""
        mission_frame = tk.LabelFrame(parent, text="Mission Control", 
                                    font=("Arial", 12, "bold"),
                                    bg='#34495e', fg='white', bd=2)
        mission_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)
        
        # Mission buttons
        btn_frame = tk.Frame(mission_frame, bg='#34495e')
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="Square Pattern", command=self.start_square_mission,
                bg='#f39c12', fg='white', font=("Arial", 10, "bold"),
                width=15, height=2).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Circle Pattern", command=self.start_circle_mission,
                bg='#f39c12', fg='white', font=("Arial", 10, "bold"),
                width=15, height=2).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Stop Mission", command=self.stop_mission,
                bg='#e74c3c', fg='white', font=("Arial", 10, "bold"),
                width=15, height=2).pack(side=tk.LEFT, padx=5)
        
        # Mission status
        self.mission_status_label = tk.Label(mission_frame, text="Mission: Standby", 
                                           bg='#34495e', fg='#ecf0f1', 
                                           font=("Arial", 11, "bold"))
        self.mission_status_label.pack(pady=5)
        
    def setup_key_bindings(self):
        """Setup keyboard bindings"""
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.focus_set()
        
        # Key states
        self.pressed_keys = set()
        
    def on_key_press(self, event):
        """Handle key press events"""
        key = event.keysym.lower()
        self.pressed_keys.add(key)
        
        # Special keys for immediate actions
        if key == 'space':
            self.takeoff()
        elif key == 'x':
            self.land()
        elif key == 'h':
            self.return_home()
        elif key == 'escape':
            self.emergency_land()
        elif key == 'c':
            self.toggle_control()
        
    def on_key_release(self, event):
        """Handle key release events"""
        key = event.keysym.lower()
        self.pressed_keys.discard(key)
        
    def on_map_click(self, event):
        """Handle map click for target setting"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        # Convert canvas coordinates to world coordinates
        x = (event.x - 250) / 10.0  # Scale: 10 pixels = 1 meter
        y = (250 - event.y) / 10.0  # Flip Y axis
        
        # Safety check
        distance = math.sqrt(x*x + y*y)
        if distance > self.max_distance:
            messagebox.showwarning("Warning", f"Target too far! Max distance: {self.max_distance}m")
            return
            
        # Set target
        self.target_position[0] = x
        self.target_position[1] = y
        
        # Publish target
        target_msg = Point()
        target_msg.x = float(x)
        target_msg.y = float(y)
        target_msg.z = float(self.target_position[2])
        self.target_publisher.publish(target_msg)
        
        self.get_logger().info(f"Target set to: ({x:.1f}, {y:.1f}, {self.target_position[2]:.1f})")
        
    def draw_map(self):
        """Draw the arena map"""
        self.map_canvas.delete("all")
        
        # Map dimensions: 50m x 50m, scaled to 500x500 pixels
        scale = 10  # 10 pixels per meter
        
        # Draw grid
        for i in range(0, 501, 50):  # Every 5 meters
            self.map_canvas.create_line(i, 0, i, 500, fill='#bdc3c7', width=1)
            self.map_canvas.create_line(0, i, 500, i, fill='#bdc3c7', width=1)
        
        # Draw center axes
        self.map_canvas.create_line(250, 0, 250, 500, fill='#34495e', width=2)
        self.map_canvas.create_line(0, 250, 500, 250, fill='#34495e', width=2)
        
        # Draw obstacles (simplified representation)
        obstacles = [
            # Main walls
            (10, 10, 40, 40, '#e74c3c'),   # Top-left corner
            (460, 10, 490, 40, '#e74c3c'), # Top-right corner
            (10, 460, 40, 490, '#e74c3c'), # Bottom-left corner
            (460, 460, 490, 490, '#e74c3c'), # Bottom-right corner
            
            # Center obstacles
            (200, 200, 220, 220, '#e67e22'),
            (280, 280, 300, 300, '#e67e22'),
            (180, 320, 200, 340, '#e67e22'),
            (320, 180, 340, 200, '#e67e22'),
        ]
        
        for x1, y1, x2, y2, color in obstacles:
            self.map_canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='black')
        
        # Draw safety boundary
        self.map_canvas.create_oval(250 - self.max_distance*scale, 
                                  250 - self.max_distance*scale,
                                  250 + self.max_distance*scale, 
                                  250 + self.max_distance*scale,
                                  outline='red', width=2, dash=(5,5))
        
        # Draw home position
        self.map_canvas.create_oval(245, 245, 255, 255, fill='#27ae60', outline='black', width=2)
        self.map_canvas.create_text(250, 240, text="HOME", fill='#27ae60', font=("Arial", 8, "bold"))
        
        # Legend
        legend_x = 10
        legend_y = 450
        self.map_canvas.create_text(legend_x, legend_y, text="Legend:", anchor="w", 
                                  fill='black', font=("Arial", 9, "bold"))
        self.map_canvas.create_rectangle(legend_x, legend_y+15, legend_x+10, legend_y+25, 
                                       fill='#e74c3c')
        self.map_canvas.create_text(legend_x+15, legend_y+20, text="Obstacles", anchor="w", 
                                  fill='black', font=("Arial", 8))
        self.map_canvas.create_oval(legend_x, legend_y+30, legend_x+10, legend_y+40, 
                                  fill='#27ae60')
        self.map_canvas.create_text(legend_x+15, legend_y+35, text="Home", anchor="w", 
                                  fill='black', font=("Arial", 8))
        
    def update_map_display(self):
        """Update drone and target positions on map"""
        scale = 10
        
        # Remove previous drone and target markers
        self.map_canvas.delete("drone")
        self.map_canvas.delete("target")
        self.map_canvas.delete("path")
        
        # Draw drone position (blue triangle)
        drone_x = 250 + self.current_position[0] * scale
        drone_y = 250 - self.current_position[1] * scale
        
        # Draw drone as triangle pointing in yaw direction
        size = 8
        angle = self.current_yaw
        points = []
        for i, (dx, dy) in enumerate([(0, -size), (-size//2, size//2), (size//2, size//2)]):
            # Rotate point
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            rx = dx * cos_a - dy * sin_a
            ry = dx * sin_a + dy * cos_a
            points.extend([drone_x + rx, drone_y + ry])
        
        self.map_canvas.create_polygon(points, fill='#3498db', outline='black', width=2, tags="drone")
        
        # Draw target position (red circle)
        target_x = 250 + self.target_position[0] * scale
        target_y = 250 - self.target_position[1] * scale
        self.map_canvas.create_oval(target_x-6, target_y-6, target_x+6, target_y+6, 
                                  fill='#e74c3c', outline='black', width=2, tags="target")
        
        # Draw path line
        self.map_canvas.create_line(drone_x, drone_y, target_x, target_y, 
                                  fill='#f39c12', width=2, dash=(3,3), tags="path")
        
        # Draw mission points if active
        if self.mission_active and self.mission_points:
            for i, point in enumerate(self.mission_points):
                px = 250 + point[0] * scale
                py = 250 - point[1] * scale
                color = '#27ae60' if i == self.current_mission_point else '#95a5a6'
                self.map_canvas.create_oval(px-4, py-4, px+4, py+4, 
                                          fill=color, outline='black', tags="mission")
        
    def toggle_control(self):
        """Toggle control enable/disable"""
        self.control_enabled = not self.control_enabled
        
        # Publish enable state
        enable_msg = Bool()
        enable_msg.data = self.control_enabled
        self.enable_publisher.publish(enable_msg)
        
        # Update button
        if self.control_enabled:
            self.control_btn.config(text="Disable Control", bg='#e74c3c')
        else:
            self.control_btn.config(text="Enable Control", bg='#27ae60')
            
        self.get_logger().info(f"Control {'enabled' if self.control_enabled else 'disabled'}")
        
    def takeoff(self):
        """Takeoff command"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        self.target_position[2] = 3.0
        target_msg = Point()
        target_msg.x = float(self.current_position[0])
        target_msg.y = float(self.current_position[1])
        target_msg.z = 3.0
        self.target_publisher.publish(target_msg)
        
        self.get_logger().info("Takeoff command sent")
        
    def land(self):
        """Landing command"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        self.target_position[2] = 0.5
        target_msg = Point()
        target_msg.x = float(self.current_position[0])
        target_msg.y = float(self.current_position[1])
        target_msg.z = 0.5
        self.target_publisher.publish(target_msg)
        
        self.get_logger().info("Landing command sent")
        
    def return_home(self):
        """Return to home position"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        self.target_position = [0.0, 0.0, 2.0]
        target_msg = Point()
        target_msg.x = 0.0
        target_msg.y = 0.0
        target_msg.z = 2.0
        self.target_publisher.publish(target_msg)
        
        self.get_logger().info("Returning home")
        
    def emergency_land(self):
        """Emergency landing"""
        self.target_position[2] = 0.0
        target_msg = Point()
        target_msg.x = float(self.current_position[0])
        target_msg.y = float(self.current_position[1])
        target_msg.z = 0.0
        self.target_publisher.publish(target_msg)
        
        # Disable control
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_publisher.publish(enable_msg)
        
        self.get_logger().warn("EMERGENCY LANDING!")
        messagebox.showwarning("Emergency", "Emergency landing activated!")
        
    def update_speed(self, value):
        """Update speed level"""
        self.speed_level = int(value)
        
    def move_direction(self, direction):
        """Move in specified direction"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        step = self.speed_level * 0.5  # 0.5 to 2.5 meters per step
        
        new_target = self.target_position.copy()
        
        if direction == 'forward':
            new_target[1] += step
        elif direction == 'backward':
            new_target[1] -= step
        elif direction == 'left':
            new_target[0] -= step
        elif direction == 'right':
            new_target[0] += step
        elif direction == 'up':
            new_target[2] = min(new_target[2] + step, self.max_altitude)
        elif direction == 'down':
            new_target[2] = max(new_target[2] - step, self.min_altitude)
        elif direction == 'turn_left':
            self.target_yaw += math.pi / 4  # 45 degrees
        elif direction == 'turn_right':
            self.target_yaw -= math.pi / 4  # 45 degrees
            
        # Safety check for position
        if direction in ['forward', 'backward', 'left', 'right']:
            distance = math.sqrt(new_target[0]**2 + new_target[1]**2)
            if distance <= self.max_distance:
                self.target_position = new_target
            else:
                messagebox.showwarning("Warning", "Movement would exceed safety boundary!")
                return
        elif direction in ['up', 'down']:
            self.target_position = new_target
            
        # Publish target
        target_msg = Point()
        target_msg.x = float(self.target_position[0])
        target_msg.y = float(self.target_position[1])
        target_msg.z = float(self.target_position[2])
        self.target_publisher.publish(target_msg)
        
    def start_square_mission(self):
        """Start square pattern mission"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        size = 5.0  # 5m square
        self.mission_points = [
            [size, 0.0, 2.0],
            [size, size, 2.0],
            [0.0, size, 2.0],
            [0.0, 0.0, 2.0]
        ]
        self.current_mission_point = 0
        self.mission_active = True
        self.mission_type = "Square Pattern"
        
        # Start mission timer
        if self.mission_timer:
            self.mission_timer.cancel()
        self.mission_timer = self.create_timer(2.0, self.mission_step)
        
        self.get_logger().info("Square mission started")
        
    def start_circle_mission(self):
        """Start circle pattern mission"""
        if not self.control_enabled:
            messagebox.showwarning("Warning", "Enable control first!")
            return
            
        radius = 5.0
        points = 8
        self.mission_points = []
        
        for i in range(points):
            angle = 2 * math.pi * i / points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            self.mission_points.append([x, y, 2.0])
            
        self.current_mission_point = 0
        self.mission_active = True
        self.mission_type = "Circle Pattern"
        
        # Start mission timer
        if self.mission_timer:
            self.mission_timer.cancel()
        self.mission_timer = self.create_timer(3.0, self.mission_step)
        
        self.get_logger().info("Circle mission started")
        
    def stop_mission(self):
        """Stop current mission"""
        self.mission_active = False
        self.mission_type = None
        if self.mission_timer:
            self.mission_timer.cancel()
            self.mission_timer = None
        self.get_logger().info("Mission stopped")
        
    def mission_step(self):
        """Execute one step of the mission"""
        if not self.mission_active or not self.mission_points:
            return
            
        # Move to next point
        point = self.mission_points[self.current_mission_point]
        self.target_position = point.copy()
        
        # Publish target
        target_msg = Point()
        target_msg.x = float(point[0])
        target_msg.y = float(point[1])
        target_msg.z = float(point[2])
        self.target_publisher.publish(target_msg)
        
        # Move to next point
        self.current_mission_point = (self.current_mission_point + 1) % len(self.mission_points)
        
    def control_loop(self):
        """Main control loop for continuous movement"""
        if not self.control_enabled:
            return
            
        # Process continuous key presses
        move_step = self.speed_level * 0.1  # Smaller steps for smooth movement
        
        new_target = self.target_position.copy()
        changed = False
        
        if 'w' in self.pressed_keys:  # Forward
            new_target[1] += move_step
            changed = True
        if 's' in self.pressed_keys:  # Backward
            new_target[1] -= move_step
            changed = True
        if 'a' in self.pressed_keys:  # Left
            new_target[0] -= move_step
            changed = True
        if 'd' in self.pressed_keys:  # Right
            new_target[0] += move_step
            changed = True
        if 'r' in self.pressed_keys:  # Up
            new_target[2] = min(new_target[2] + move_step, self.max_altitude)
            changed = True
        if 'f' in self.pressed_keys:  # Down
            new_target[2] = max(new_target[2] - move_step, self.min_altitude)
            changed = True
        if 'q' in self.pressed_keys:  # Turn left
            self.target_yaw += 0.05
        if 'e' in self.pressed_keys:  # Turn right
            self.target_yaw -= 0.05
            
        # Safety check and update
        if changed:
            distance = math.sqrt(new_target[0]**2 + new_target[1]**2)
            if distance <= self.max_distance:
                self.target_position = new_target
                
                # Publish target
                target_msg = Point()
                target_msg.x = float(self.target_position[0])
                target_msg.y = float(self.target_position[1])
                target_msg.z = float(self.target_position[2])
                self.target_publisher.publish(target_msg)
                
    def update_gui(self):
        """Update GUI displays"""
        try:
            # Update status labels
            self.status_labels['current_pos'].config(
                text=f"({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})")
            self.status_labels['target_pos'].config(
                text=f"({self.target_position[0]:.1f}, {self.target_position[1]:.1f}, {self.target_position[2]:.1f})")
            self.status_labels['altitude'].config(text=f"{self.current_position[2]:.1f}m")
            self.status_labels['yaw'].config(text=f"{math.degrees(self.current_yaw):.0f}°")
            self.status_labels['speed'].config(text=f"Level {self.speed_level}")
            self.status_labels['control'].config(
                text="ENABLED" if self.control_enabled else "DISABLED",
                fg='#27ae60' if self.control_enabled else '#e74c3c')
            
            mission_text = self.mission_type if self.mission_active else "Standby"
            self.status_labels['mission'].config(text=mission_text)
            
            # Distance calculations
            dist_home = math.sqrt(sum(x**2 for x in self.current_position[:2]))
            dist_target = math.sqrt(sum((c-t)**2 for c, t in zip(self.current_position, self.target_position)))
            
            self.status_labels['dist_home'].config(text=f"{dist_home:.1f}m")
            self.status_labels['dist_target'].config(text=f"{dist_target:.1f}m")
            
            # Safety status
            safety_status = "OK"
            safety_color = '#27ae60'
            
            if dist_home > self.max_distance * 0.8:
                safety_status = "WARNING"
                safety_color = '#f39c12'
            if dist_home > self.max_distance:
                safety_status = "DANGER"
                safety_color = '#e74c3c'
                
            self.status_labels['safety'].config(text=safety_status, fg=safety_color)
            
            # Update mission status
            mission_status = f"Mission: {mission_text}"
            if self.mission_active:
                mission_status += f" (Point {self.current_mission_point + 1}/{len(self.mission_points)})"
            self.mission_status_label.config(text=mission_status)
            
            # Update map display
            self.update_map_display()
            
        except Exception as e:
            self.get_logger().error(f"GUI update error: {e}")
            
    def on_closing(self):
        """Handle window closing"""
        self.cleanup()
        
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info("Shutting down GUI system...")
        
        # Stop mission
        self.stop_mission()
        
        # Disable control
        try:
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_publisher.publish(enable_msg)
        except:
            pass
            
        # Destroy GUI
        try:
            self.root.quit()
            self.root.destroy()
        except:
            pass

def check_ros_connection():
    """Check ROS2 connection status"""
    try:
        import subprocess
        import time
        
        # Check ros2 topic list command
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"✅ ROS2 connection OK - Found {len(topics)} topics")
            
            # Check for drone-specific topics
            drone_topics = [t for t in topics if 'ppo' in t.lower() or 'drone' in t.lower()]
            if drone_topics:
                print("✅ Drone topics found:", drone_topics)
            else:
                print("⚠️  No drone-specific topics found")
                print("   Make sure to run 1_start_system.bat and 2_takeoff.bat first")
            return True
        else:
            print("❌ ROS2 connection failed")
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ ROS2 connection timeout")
        return False
    except FileNotFoundError:
        print("❌ ROS2 not found - make sure ROS2 is installed and sourced")
        return False
    except Exception as e:
        print(f"❌ ROS2 connection error: {e}")
        return False

def main():
    """Main function"""
    print("🚁 Drone GUI Teleoperation System Starting...")
    print("=" * 50)
    
    # Check ROS2 connection
    if not check_ros_connection():
        print("\n💡 Troubleshooting:")
        print("1. Make sure WSL2 is running")
        print("2. Source ROS2: source /opt/ros/humble/setup.bash")
        print("3. Run 1_start_system.bat first")
        print("4. Run 2_takeoff.bat to enable drone control")
        print("\nThe GUI will still start but may not connect to the drone.")
        
        # Ask user if they want to continue
        try:
            response = input("\nContinue anyway? (y/n): ").lower()
            if response != 'y':
                return
        except:
            pass
    
    # Initialize ROS2
    try:
        rclpy.init()
    except Exception as e:
        print(f"❌ Failed to initialize ROS2: {e}")
        return
    
    try:
        print("🖥️  Starting GUI...")
        
        # Create GUI controller
        controller = DroneGUIController()
        
        # ROS2 spin in separate thread
        ros_thread = threading.Thread(target=lambda: rclpy.spin(controller))
        ros_thread.daemon = True
        ros_thread.start()
        
        print("✅ GUI started successfully!")
        print("\n🎮 Controls:")
        print("WASD: Move Forward/Left/Backward/Right")
        print("RF: Move Up/Down")
        print("QE: Turn Left/Right")
        print("Space: Takeoff")
        print("X: Land")
        print("H: Return Home")
        print("C: Toggle Control")
        print("ESC: Emergency Land")
        print("\n🖱️  Click on map to set target position")
        print("📊 Real-time status in right panel")
        print("🎯 Use mission buttons for automated patterns")
        
        # Start Tkinter main loop
        controller.root.mainloop()
        
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            controller.cleanup()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("✅ System shutdown complete")

if __name__ == '__main__':
    main() 