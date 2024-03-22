#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, LockDraw
import re
import numpy as np
import transforms3d as tr
import math
import csv
from scipy.interpolate import interp1d
from pyquaternion import Quaternion
import matplotlib.colors as mcolors




class Plotter(Node):

    def __init__(self):

        super().__init__('plotter')

        self.declare_parameter('folder_name', '5Hz-RP')
        self.declare_parameter('truncate', 0)

        folder_name = self.get_parameter('folder_name').value
        self.runs_filepath = os.path.join(get_package_share_directory('simulation_bringup'), 'data', folder_name)
        self.gt_filepath = os.path.join(get_package_share_directory('simulation_bringup'), 'data', folder_name, 'GT.txt')


        self.declare_parameter('order_by_range', True)

        
        #obsolete; to refactor
        self.limit = 5000


        if self.get_parameter('truncate').value == 0:

            self.plot_limiter = 3000
            self.path_limiter = -1

        else:

            self.plot_limiter = self.get_parameter('truncate').value
            self.path_limiter = self.get_parameter('truncate').value
        

        self.gt_x = np.empty(0)
        self.gt_y = np.empty(0)
        self.gt_yaw = np.empty(0)
        self.gt_timestamps = np.empty(0)
        self.gt_quat = np.empty((0,4))

        self.gt_x_func = None
        self.gt_y_func = None
        self.gt_yaw_func = None
        


        self.parse_gt_data()


        self.rmse_x_non_interp = {}
        self.rmse_y_non_interp = {}
        self.yaw_rmse_non_interp = {}

        self.est_x_data = {}
        self.est_y_data = {}
        self.est_yaw_data = {}
        self.est_quat_data = {}
        self.est_timestamps_data = {}

        self.gt_x_plot_data = None
        self.gt_y_plot_data = None
        self.gt_yaw_plot_data = None

        self.est_x_plot_data = {}
        self.est_y_plot_data = {}
        self.est_yaw_plot_data = {}


        self.rmse_x_data = {}
        self.rmse_y_data = {}
      
        self.yaw_rmse_data = {}

        files = [os.path.join(self.runs_filepath, file) for file in os.listdir(self.runs_filepath)]
        for file in files:

            if file.endswith('txt') and not os.path.basename(file) == 'GT.txt':
         
                self.parse_path_message(file)

        self.calculate_rmse()
        

        self.figure = plt.figure()
        self.subfigures = self.figure.subplots(ncols=2, nrows=2)
        self.colors = {}
        self.points = []


        self.lines = []
        self.lines_dict = {}

        

        self.plot_path(0, 0)
        # self.plot_rmse_x()
        # self.plot_rmse_y()
        # self.plot_yaw_rmse(0,2)
        self.plot_hist_x(0,1)
        self.plot_hist_y(1, 0)
        self.plot_hist_yaw(1,1)
        

        self.map_legend_to_ax = {}
        for legend_line, ax_line in zip(self.leg.get_patches(), self.lines_dict.keys()):
            
            legend_line.set_picker(6)  # Enable picking on the legend line.

            
            self.map_legend_to_ax[legend_line] = ax_line
        


        # mng = plt.get_current_fig_manager()
        # mng.resize(*mng.window.maxsize())

     

        self.figure.canvas.mpl_connect('pick_event', self.on_legend_click)
        
        plt.subplots_adjust(0.034, 0.036, 0.991, 0.951, 0.251, 0.23)
        plt.show()


    def sort_keys(self):

        default_colors = list(mcolors.TABLEAU_COLORS.values())

        range_first = self.get_parameter('order_by_range').value

        keys = list(self.rmse_y_data.keys())

        distances = np.array([float(os.path.basename(key).strip('.txt').split('-')[0 if range_first else 1]) for key in keys])
        
        indexes_sorted_by_distance = np.argsort(distances)

        sorted_keys = [list(self.rmse_y_data.keys())[i] for i in indexes_sorted_by_distance]

        samples = np.array([float(os.path.basename(key).strip('.txt').split('-')[1 if range_first else 0]) for key in sorted_keys])

        rep = np.unique(samples).shape[0]     

        colors_to_use = default_colors[:rep]


        names = []
        colors = []



        for i in range(0, int(samples.shape[0]/rep)):
            
            argsort_samples = np.argsort(samples[rep*(i):rep*(i+1)])
            tmp = [sorted_keys[rep*(i):rep*(i+1)][j] for j in argsort_samples]

            names.append(tmp)
            colors.append(colors_to_use)
        
        names = np.array(names).flatten()
        colors = np.array(colors).flatten()

        return names, colors
    


    def on_legend_click(self, event):



        legend_line = event.artist
      
        if legend_line not in self.map_legend_to_ax:
            return
        
        key = self.map_legend_to_ax[legend_line]
        for ax_line in self.lines_dict[key]:

            visible = not ax_line.get_visible()
            # print(visible)
            ax_line.set_visible(visible)
        legend_line.set_alpha(1.0 if visible else 0.2)
        self.figure.canvas.draw()
        # plt.draw()
      

    def plot_hist_x(self, row, col):

        self.hist_x_ax = self.subfigures[row, col]

        
        # names = np.array([float(os.path.basename(key).strip('.txt').split('-')[0]) for key in self.rmse_x_data.keys()])
        # sorted_indexes = np.argsort(names)
        # sorted_keys = [list(self.rmse_x_data.keys())[i] for i in sorted_indexes]

        sorted_keys, colors = self.sort_keys()

        final_values = []

        for i, key in enumerate(sorted_keys):

            line = self.hist_x_ax.bar(i, self.rmse_x_data[key][-1], label = f'{self.get_label(key)}', color = colors[i])
            final_values.append(self.rmse_x_data[key][-1])

            self.lines_dict[key].append(line[0])

        self.hist_x_ax.plot(range(0,len(final_values)), final_values, color = 'black')
        self.hist_x_ax.scatter(np.argmin(final_values), final_values[np.argmin(final_values)], color = 'black', marker = 'o', linewidths = 5)


        self.hist_x_ax.set_xlabel('Simulations')
        self.hist_x_ax.set_ylabel('RMSE X')
        self.hist_x_ax.set_title("RMSE X")
        self.hist_x_ax.grid()
        # self.hist_x_ax.legend().set_draggable(True)


    def plot_hist_y(self, row, col):

        self.hist_y_ax = self.subfigures[row, col]

        self.sort_keys()

        # names = np.array([float(os.path.basename(key).strip('.txt').split('-')[1]) for key in self.rmse_y_data.keys()])
        # sorted_indexes = np.argsort(names)
        # sorted_keys = [list(self.rmse_y_data.keys())[i] for i in sorted_indexes]

        final_values = []

        sorted_keys, colors = self.sort_keys()

        for i, key in enumerate(sorted_keys):

            line = self.hist_y_ax.bar(i, self.rmse_y_data[key][-1], label = f'{self.get_label(key)}', color = colors[i])
            final_values.append(self.rmse_y_data[key][-1])

            self.lines_dict[key].append(line[0])

        self.hist_y_ax.plot(range(0,len(final_values)), final_values, color = 'black')
        self.hist_y_ax.scatter(np.argmin(final_values), final_values[np.argmin(final_values)], color = 'black', marker = 'o', linewidths = 5)


        self.hist_y_ax.set_xlabel('Simulations')
        self.hist_y_ax.set_ylabel('RMSE Y')
        self.hist_y_ax.set_title("RMSE Y")
        self.hist_y_ax.grid()
        # self.hist_y_ax.legend().set_draggable(True)

    def plot_hist_yaw(self, row, col):

            self.hist_yaw_ax = self.subfigures[row, col]

            sorted_keys, colors = self.sort_keys()

            final_values = []

            for i, key in enumerate(sorted_keys):

                line = self.hist_yaw_ax.bar(i, self.yaw_rmse_data[key][-1], label = f'{self.get_label(key)}', color = colors[i])
                final_values.append(self.yaw_rmse_data[key][-1])

                self.lines_dict[key].append(line[0])


            self.hist_yaw_ax.plot(range(0,len(final_values)), final_values, color = 'black')
            self.hist_yaw_ax.scatter(np.argmin(final_values), final_values[np.argmin(final_values)], color = 'black', marker = 'o', linewidths = 5)


            self.hist_yaw_ax.set_xlabel('Simulations')
            self.hist_yaw_ax.set_ylabel('RMSE Yaw')
            self.hist_yaw_ax.set_title("RMSE Yaw")
            self.hist_yaw_ax.grid()
            self.leg = self.hist_yaw_ax.legend(fancybox=True, shadow=True, loc = 'upper left')
            self.leg.set_draggable(True)


    def plot_path(self, row, col):

        self.path_ax = self.subfigures[row,col]
        
  

        #non interpolated
        # self.lines.append(self.path_ax.plot(self.gt_x, self.gt_y, label='GT Path', color = 'black')[0])   

        sorted_keys, colors = self.sort_keys()

        for i, key in enumerate(sorted_keys):    

            (line, ) = self.path_ax.plot(self.est_x_plot_data[key][:self.path_limiter], self.est_y_plot_data[key][:self.path_limiter], label=f'{self.get_label(key)}', color = colors[i])
        

            #non interpolated
            # (line, ) = self.path_ax.plot(self.est_x_data[key], self.est_y_data[key], label=f'{self.get_label(key)}')
            self.lines.append(line)
            self.lines_dict[key] = [line]
            self.colors[key] = line.get_color()


        self.lines.append(self.path_ax.plot(self.gt_x_plot_data, self.gt_y_plot_data, label='GT Path', color = 'black', linewidth = 2)[0])

        self.path_ax.set_xlabel('X')
        self.path_ax.set_ylabel('Y')
        self.path_ax.set_title('Paths')
        self.path_ax.grid()
        # self.leg = self.path_ax.legend(fancybox=True, shadow=True, loc = 'upper left')
        # self.leg.set_draggable(True)


    def plot_rmse_x(self, row, col):

        self.rmse_x_ax = self.subfigures[row,col]
        

        for i, key in enumerate(self.rmse_x_data.keys()):
            
            line = self.rmse_x_ax.plot(range(0, len(self.rmse_x_data[key][:self.limit])), self.rmse_x_data[key][:self.limit], label=f'{self.get_label(key)}', )
            self.lines_dict[key].append(line[0])
            line[0].set_color(self.colors[key])

        
        self.rmse_x_ax.set_xlabel('PoseStamped received')
        self.rmse_x_ax.set_ylabel('X RMSE')
        self.rmse_x_ax.set_title('RMSE X')
        self.rmse_x_ax.grid()
        self.rmse_x_ax.legend().set_draggable(True)


    def plot_rmse_y(self, row, col):

        self.rmse_y_ax = self.subfigures[row,col]
        

        for i, key in enumerate(self.rmse_x_data.keys()):
            
            line = self.rmse_y_ax.plot(range(0, len(self.rmse_y_data[key][:self.limit])), self.rmse_y_data[key][:self.limit], label=f'{self.get_label(key)}', )
            self.lines_dict[key].append(line[0])
            line[0].set_color(self.colors[key])

        self.rmse_y_ax.set_xlabel('PoseStamped received')
        self.rmse_y_ax.set_ylabel('Y RMSE')
        self.rmse_y_ax.set_title('RMSE Y')
        self.rmse_y_ax.grid()
        self.rmse_y_ax.legend().set_draggable(True)


    def plot_yaw_rmse(self, row, col):

        self.yaw_rmse_ax = self.subfigures[row,col]
        

        for i, key in enumerate(self.yaw_rmse_data.keys()):

            line = self.yaw_rmse_ax.plot(range(0, len(self.yaw_rmse_data[key][:self.limit])), self.yaw_rmse_data[key][:self.limit], label=f'{self.get_label(key)}', )
            self.lines_dict[key].append(line[0])
            line[0].set_color(self.colors[key])

        self.yaw_rmse_ax.set_xlabel('PoseStamped received')
        self.yaw_rmse_ax.set_ylabel('Yaw RMSE')
        self.yaw_rmse_ax.set_title('Yaw RMSE')   
        self.yaw_rmse_ax.grid()
        self.yaw_rmse_ax.legend().set_draggable(True)

    def to_float(self, s):
        # Regular expression pattern for scientific notation
        # scientific_notation_pattern = r'^(\d+\.\d+)e-(\d+)$'
        scientific_notation_pattern = r'^(-?\d+\.\d+)e-(\d+)$'

        

        # Check if the string matches the scientific notation pattern
        match = re.match(scientific_notation_pattern, s)
        if match:

            assert('e' in s)
            # If it matches, extract mantissa and exponent
            mantissa = float(match.group(1))
            exponent = int(match.group(2))

            # Calculate the value using mantissa and exponent
            return mantissa * 10**(-exponent)
        else:
            # If it doesn't match, just convert the string to float directly
            assert(not 'e' in s)
            return float(s)

    def parse_gt_data(self):


        with open(self.gt_filepath, 'r') as file:

            header_lines = True
            pose_lines = False
            header_strings = ''
            pose_strings = ''          

            #Separate Header strings from Pose strings
            for line in file.readlines():
                
                if header_lines:                    
                    
                    header_strings += line
                    if line == 'Pose:\n':

                        header_strings.strip(line)
                        header_lines = False
                        pose_lines = True
                        # self.get_logger().warn(line)

                if pose_lines:
                    
                    pose_strings+=line

        #Parse data to get x, y and yaw
        for i, line in enumerate(pose_strings.split('Pose:\n')):

            if i>0:

                line = line.strip('geometry_msgs.msg.PoseStamped')
                tags = line.split('),')
                sec = int(re.search(r"sec=(\d+)", tags[0]).group(1))
                nanosec = int(re.search(r"nanosec=(\d+)", tags[0]).group(1))
                frame_id = re.search(r"frame_id='([^']*)'", tags[1]).group(1)
                point = tags[2].strip(' pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(')
                quaternion = tags[3].strip(' orientation=geometry_msgs.msg.Quaternion(').strip(')))\n')
                # point_x = float(re.search(r"x=([-+]?\d*\.\d+)", point).group(1))
                # point_y = float(re.search(r"y=([-+]?\d*\.\d+)", point).group(1))

                point_x_string = re.search(r'x=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', point).group(1)
                point_y_string = re.search(r'y=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', point).group(1)

                point_x = self.to_float(point_x_string)
                point_y = self.to_float(point_y_string)

                
                orientation_x = float(re.search(r"x=([-+]?\d*\.\d+)", quaternion).group(1))
                orientation_y = float(re.search(r"y=([-+]?\d*\.\d+)", quaternion).group(1))
                orientation_z = float(re.search(r"z=([-+]?\d*\.\d+)", quaternion).group(1))
                orientation_w = float(re.search(r"w=([-+]?\d*\.\d+)", quaternion).group(1))

                orientation_x_string = re.search(r'x=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                orientation_y_string = re.search(r'y=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                orientation_z_string = re.search(r'z=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                orientation_w_string = re.search(r'w=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                orientation_x = self.to_float(orientation_x_string)
                orientation_y = self.to_float(orientation_y_string)
                orientation_z = self.to_float(orientation_z_string)
                orientation_w = self.to_float(orientation_w_string)


                time = sec + (1000 * nanosec)
                self.gt_timestamps = np.append(self.gt_timestamps, time)

                self.gt_x = np.append(self.gt_x, point_x)
                self.gt_y = np.append(self.gt_y, point_y)

                


                quat = Quaternion([orientation_w, orientation_x, orientation_y, orientation_z])

                # quat = quat + tr.euler.euler2quat(0,0,180)
  

                yaw = tr.euler.quat2euler(quat, axes='sxyz')[2]


                self.gt_quat = np.append(self.gt_quat, quat)

                

                self.gt_yaw = np.append(self.gt_yaw, yaw)
                    

        t = np.linspace(0, 1, len(self.gt_y)) 

        self.gt_x_func = interp1d(t, self.gt_x, kind='linear')
        self.gt_y_func = interp1d(t, self.gt_y, kind='linear')
        self.gt_yaw_func = interp1d(t, self.gt_yaw, kind='linear')


    def calculate_rmse(self):

        for key in self.est_x_data.keys():     

            self.rmse_x_data[key] = np.empty(0)
            self.rmse_y_data[key] = np.empty(0)
            self.yaw_rmse_data[key] = np.empty(0)

            self.rmse_x_non_interp[key] = np.empty(0)
            self.rmse_y_non_interp[key] = np.empty(0)
            self.yaw_rmse_non_interp[key] = np.empty(0)

            
            t = np.linspace(0, 1, len(self.est_x_data[key]))  # Normalized x values for array1      
         
            est_x_func = interp1d(t, self.est_x_data[key], kind='linear')
            est_y_func = interp1d(t, self.est_y_data[key], kind='linear')
            est_yaw_func = interp1d(t, self.est_yaw_data[key], kind='linear')

            new_t = np.linspace(0, 1, max(len(self.est_x_data[key]), len(self.gt_x)))

            est_x = est_x_func(new_t)
            est_y = est_y_func(new_t)
            est_yaw = est_yaw_func(new_t)

            gt_x = self.gt_x_func(new_t)
            gt_y = self.gt_y_func(new_t)
            gt_yaw = self.gt_yaw_func(new_t)


            self.gt_x_plot_data = gt_x
            self.gt_y_plot_data = gt_y
            self.gt_yaw_plot_data = gt_yaw

            self.est_x_plot_data[key] = est_x
            self.est_y_plot_data[key] = est_y
            self.est_yaw_plot_data[key] = est_yaw



            for i in range(1, est_x.shape[0]):

        #         # print(self.gt_x[:i].shape)
        #         # print(self.est_x_data[key][:1].shape)

                # err = self.gt_quat[:i]-self.est_quat_data[key][:i]

                # err = np.array([tr.euler.quat2euler(quat, axes='sxyz')[2] for quat in err])
                # print(self.est_quat_data[key][i].degrees)  


                mse_x = np.mean((gt_x[:i] - est_x[:i])**2)
                mse_y = np.mean((gt_y[:i] - est_y[:i])**2)
                yaw_mse = np.mean((gt_yaw[:i] - est_yaw[:i])**2)
                
                # yaw_mse = np.mean(err**2)

                mse_x_non_interp = np.mean((self.gt_x[:i] - self.est_x_data[key][:i])**2)
                mse_y_non_interp = np.mean((self.gt_y[:i] - self.est_y_data[key][:i])**2)
                yaw_mse_non_interp = np.mean((self.gt_yaw[:i] - self.est_yaw_data[key][:i])**2)



                rmse_x = np.sqrt(mse_x)
                rmse_y = np.sqrt(mse_y)
                yaw_rmse = np.sqrt(yaw_mse) * 180/np.pi


                rmse_x_non_interp = np.sqrt(mse_x_non_interp)
                rmse_y_non_interp = np.sqrt(mse_y_non_interp)
                yaw_rmse_non_interp = np.sqrt(yaw_mse_non_interp)


                self.rmse_x_non_interp[key] = np.append(self.rmse_x_non_interp[key], rmse_x_non_interp)
                self.rmse_y_non_interp[key] = np.append(self.rmse_y_non_interp[key], rmse_y_non_interp)
                self.yaw_rmse_non_interp[key] = np.append(self.yaw_rmse_non_interp[key], yaw_rmse_non_interp)


                
                if i < self.plot_limiter:

                    self.rmse_x_data[key] = np.append(self.rmse_x_data[key], rmse_x)
                    self.rmse_y_data[key] = np.append(self.rmse_y_data[key], rmse_y)
                    self.yaw_rmse_data[key] = np.append(self.yaw_rmse_data[key], yaw_rmse)



        # self.get_logger().debug(f'{self.rmse_x_data.keys()}')
                

    def get_label(self, filename):

        name = os.path.basename(filename).strip('.txt').split('-')

        # print(name)
        # seq = name[0]
        # precision = math.degrees(np.pi / int(name[2]))
        precision = 360/float(name[1])
        max_r = float(name[0])

        key = f'P={precision}° - MR={max_r}m'

        return key


    def parse_path_message(self, filepath):

        with open(filepath, 'r') as file:

            header_lines = True
            pose_lines = False
            header_strings = ''
            pose_strings = ''

            key = file.name

            self.est_timestamps_data[key] = np.empty(0)
            self.est_x_data[key] = np.empty(0)
            self.est_y_data[key] = np.empty(0)
            self.est_yaw_data[key] = np.empty(0)
            self.est_quat_data[key] = np.empty(0)

            for line in file.readlines():
                
                if header_lines:                    
                    
                    header_strings += line
                    if line == 'Pose:\n':

                        header_strings.strip(line)
                        header_lines = False
                        pose_lines = True

                if pose_lines:
                    
                    pose_strings+=line


            for i, line in enumerate(pose_strings.split('Pose:\n')):

                if i>0:

                    line = line.strip('geometry_msgs.msg.PoseStamped')
                    tags = line.split('),')
                    sec = int(re.search(r"sec=(\d+)", tags[0]).group(1))
                    nanosec = int(re.search(r"nanosec=(\d+)", tags[0]).group(1))
                    frame_id = re.search(r"frame_id='([^']*)'", tags[1]).group(1)
                    point = tags[2].strip(' pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(')
                    quaternion = tags[3].strip(' orientation=geometry_msgs.msg.Quaternion(').strip(')))\n')
                    # point_x = float(re.search(r"x=([-+]?\d*\.\d+)", point).group(1))
                    # point_y = float(re.search(r"y=([-+]?\d*\.\d+)", point).group(1))

                    point_x_string = re.search(r'x=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', point).group(1)
                    point_y_string = re.search(r'y=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', point).group(1)
                    point_x = self.to_float(point_x_string)
                    point_y = self.to_float(point_y_string)

                    # orientation_z = float(re.search(r"z=([-+]?\d*\.\d+)", quaternion).group(1))
                    # orientation_w = float(re.search(r"w=([-+]?\d*\.\d+)", quaternion).group(1))

                    orientation_z_string = re.search(r'z=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                    orientation_w_string = re.search(r'w=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', quaternion).group(1)
                    orientation_z = self.to_float(orientation_z_string)
                    orientation_w = self.to_float(orientation_w_string)

                    time = sec + (1000 * nanosec)
                    self.est_timestamps_data[key] = np.append(self.est_timestamps_data[key], time)

                    self.est_x_data[key] = np.append(self.est_x_data[key], point_x)
                    self.est_y_data[key] = np.append(self.est_y_data[key], point_y)

                    quat = Quaternion([orientation_w, 0.0, 0.0, orientation_z])

                    # quat = quat + tr.euler.euler2quat(0,0,180)

                    yaw = tr.euler.quat2euler(quat, axes='sxyz')[2]

                    

                    self.est_quat_data[key] = np.append(self.est_quat_data[key], quat)

                    self.est_yaw_data[key] = np.append(self.est_yaw_data[key], yaw)
                    

    
                    



    



def main():

    rclpy.init()

    node = Plotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()