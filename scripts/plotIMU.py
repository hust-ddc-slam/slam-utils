
import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu

# Function to read IMU data from a rosbag file
def read_imu_data(bag_file):
    accel_data = {'x': [], 'y': [], 'z': [], 'time': []}
    ang_vel_data = {'x': [], 'y': [], 'z': [], 'time': []}

    bag = rosbag.Bag(bag_file)
    for topic, msg, t in bag.read_messages(topics=['/imu/data']):
        if isinstance(msg, Imu):
            accel_data['x'].append(msg.linear_acceleration.x)
            accel_data['y'].append(msg.linear_acceleration.y)
            accel_data['z'].append(msg.linear_acceleration.z)
            accel_data['time'].append(t.to_sec())

            ang_vel_data['x'].append(msg.angular_velocity.x)
            ang_vel_data['y'].append(msg.angular_velocity.y)
            ang_vel_data['z'].append(msg.angular_velocity.z)
            ang_vel_data['time'].append(t.to_sec())
    
    bag.close()
    return accel_data, ang_vel_data

# Function to plot IMU data
def plot_imu_data(accel_data, ang_vel_data):
    fig, axs = plt.subplots(2, 1, figsize=(12, 8))

    # Plot linear acceleration
    axs[0].plot(accel_data['time'], accel_data['x'], 'r', label='X-axis')
    axs[0].plot(accel_data['time'], accel_data['y'], 'g', label='Y-axis')
    axs[0].plot(accel_data['time'], accel_data['z'], 'b', label='Z-axis')
    axs[0].set_title('Linear Acceleration')
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Acceleration [m/s^2]')
    axs[0].legend()

    # Plot angular velocity
    axs[1].plot(ang_vel_data['time'], ang_vel_data['x'], 'r', label='X-axis')
    axs[1].plot(ang_vel_data['time'], ang_vel_data['y'], 'g', label='Y-axis')
    axs[1].plot(ang_vel_data['time'], ang_vel_data['z'], 'b', label='Z-axis')
    axs[1].set_title('Angular Velocity')
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Angular Velocity [rad/s]')
    axs[1].legend()

    plt.tight_layout()
    plt.show()

# Main function
if __name__ == "__main__":
    bag_file = '/home/larry/data/hongjing_step/test1.bag'
    accel_data, ang_vel_data = read_imu_data(bag_file)
    plot_imu_data(accel_data, ang_vel_data)
