from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics, State
from time import sleep, time
import math
import matplotlib.pyplot as plt

def main():
    # Initialize BeamNGpy instance
    bng = BeamNGpy('localhost', 64256, home='C:\\Users\\ADMIN\\Desktop\\beamng\\BeamNG.tech.v0.31.2.0')
    bng.open()

    # Create a new scenario in the 'conklin_testing_facility' map named 'straight_line_test'
    scenario = Scenario('conklin_testing_facility', 'straight_line_test')

    # Create a moving vehicle
    vehicle = Vehicle('ego_vehicle', model='crash_cart', license='UTEM')
    scenario.add_vehicle(vehicle, pos=(331, 259.865, 513.890), rot_quat=(1, 1, -180, 1))

    # Create a static vehicle
    static_vehicle = Vehicle('static_vehicle', model='deformablebarrier', license='')
    scenario.add_vehicle(static_vehicle, pos=(330.07, 347.660, 512.396), rot_quat=(1, 1, 60, 1))

    # Add electrics sensor to the vehicle
    electrics_sensor = Electrics()
    vehicle.attach_sensor('electrics', electrics_sensor)

    # Place files defining our scenario for the simulator to read
    scenario.make(bng)

    # Load and start our scenario
    bng.scenario.load(scenario)
    bng.scenario.start()

    # Desired speed in m/s (50 km/h)
    target_speed_mps = 50 / 3.6

    # Control parameters
    k_p = 0.1  # Proportional gain for speed control

    def get_distance(pos1, pos2):
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)

    target_position = (330.07, 347.660, 512.396)
    collision_distance_threshold = 5.0  # Adjust as necessary

    # Data collection lists
    timestamps = []
    positions = []
    speeds = []
    acc_smooth = []
    ang_vel_smooth = []

    start_time = time()

    for _ in range(1000):  # Keep the vehicle driving longer
        sleep(0.1)  # Let the vehicle drive for a short while
        current_time = time() - start_time
        timestamps.append(current_time)

        vehicle.poll_sensors()
        state = vehicle.state
        pos = state['pos']
        speed = state['vel']
        positions.append(pos)
        speeds.append(speed)

        # Simulate IMU sensor data from state sensor for demonstration
        # IMU data collection will need actual sensor support
        acc_smooth.append(state['vel'])  # Using velocity as a proxy for acceleration
        ang_vel_smooth.append(state['rotation'])  # Using rotation as a proxy for angular velocity

        # Calculate the current speed in m/s
        current_speed_mps = math.sqrt(speed[0]**2 + speed[1]**2 + speed[2]**2)

        # Adjust the throttle based on the speed error
        speed_error = target_speed_mps - current_speed_mps
        throttle = max(0.0, min(0.5 + k_p * speed_error, 1.0))  # Ensure throttle is within valid range

        vehicle.control(throttle=throttle, steering=0)
        
        print(f"Time: {current_time:.2f}s, Position: {pos}, Speed: {current_speed_mps * 3.6:.2f} km/h, Throttle: {throttle}")

        if get_distance(pos, target_position) <= collision_distance_threshold:
            print("Vehicle has hit the static vehicle.")
            break

    # Stop the vehicle
    vehicle.control(throttle=0, brake=1)
    
    # Close the connection to the simulator
    bng.close()

    # Plot the collected data
    # Convert lists to arrays for easier indexing
    speeds = [math.sqrt(v[0]**2 + v[1]**2 + v[2]**2) * 3.6 for v in speeds]  # Convert speed to km/h
    acc_smooth = [a[2] for a in acc_smooth]  # Extract vertical acceleration (Z-axis)
    ang_vel_smooth = [a[1] for a in ang_vel_smooth]  # Extract yaw rate (Y-axis)

    plt.figure(figsize=(12, 8))

    # Plot speed over time
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, speeds, label='Speed (km/h)')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (km/h)')
    plt.title('Vehicle Speed Over Time')
    plt.legend()
    plt.grid(True)

    # Plot vertical acceleration over time
    plt.subplot(3, 1, 2)
    plt.plot(timestamps, acc_smooth, label='Vertical Acceleration (m/s^2)', color='r')
    plt.xlabel('Time (s)')
    plt.ylabel('Vertical Acceleration (m/s^2)')
    plt.title('Vertical Acceleration Over Time')
    plt.legend()
    plt.grid(True)

    # Plot yaw rate over time
    plt.subplot(3, 1, 3)
    plt.plot(timestamps, ang_vel_smooth, label='Yaw Rate (rad/s)', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (rad/s)')
    plt.title('Yaw Rate Over Time')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
