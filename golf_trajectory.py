#   Golf Shot Calculator




# Dependencies
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# I am working on this feature 
"""#   Atmospheric Conditions
temperature = 25    # degrees centigrade
humidity = 50       # percent"""


#   Constants
cd = 0.35                           # Coefficient of drag
rho = 1.225                         # kgm^-3 density of air
csa = 0.001432                      # m^2 of a regulation golf ball
a_grav = -9.81                      # ms^-2
mass_of_projectile = 0.04593        # kg of a regulation golf ball
magnus_coefficient = 0.00002        # Required for magnus effect calculations


# Creating the golf ball object
class golf_ball:
    def __init__(self , name , launch_angle_degrees , aim_angle_degrees , launch_velocity , backspin_rpm , sidespin_rpm):

        # Calculate the components of the velocity
        
        # -  Convert angles from degrees to radians
        launch_angle_radians = math.radians(launch_angle_degrees)
        aim_angle_radians = math.radians(aim_angle_degrees)

        # - Calculate the components of the velocity
        v_x = launch_velocity * math.cos(launch_angle_radians) * math.cos(aim_angle_radians) # side to side - x

        v_y = launch_velocity * math.cos(launch_angle_radians) * math.sin(aim_angle_radians) # down range - y 

        v_z =  launch_velocity * math.sin(launch_angle_radians)    # up - z


        # Ball Properties
        self.name   = name
        self.cd     = 0.35 # Coefficient of drag
        self.csa    = 0.001432 # m^2
        self.mass   = 0.04593 # kg

        # Ball Launch Properties
        self.launch_vx = v_x
        self.launch_vy = v_y
        self.launch_vz = v_z
        self.launch_x = 0
        self.launch_y = 0
        self.launch_z = 0

        # Empty arrays for storing velocity and position data
        self.x_pos   = []
        self.y_pos   = []
        self.z_pos   = []
        self.v_x     = []   # m/s
        self.v_y     = []   # m/s
        self.v_z     = []   # m/s

        self.backspin  = backspin_rpm  # rpm     positive = backspin, make negative for topspin
        self.sidespin  = sidespin_rpm  # rpm     positive = draw, make negative for fade


    def get_xyz_data(self):

        z_list = self.z_pos
        deleted_item = z_list.pop
        return z_list




    def __str__(self):
        
        # Ball flight information when called
        return f"{self.name} was hit to an xy position of {round(np.max(self.x_pos),2)}m {round(np.max(self.y_pos),2)}m, and had a max altitude of {round(np.max(self.z_pos),2)}m"    


# Hits the ball given the ball object
def hit_golf_ball(ball):

    dt = 0.0001
    accuracy = 5
    timestep_count = 0

    # Initial Positions and Velocities
    x, y, z = ball.launch_x, ball.launch_y, ball.launch_z
    v_x, v_y, v_z = ball.launch_vx, ball.launch_vy, ball.launch_vz

    while z >= 0:
        # Append current positions to the ball's trajectory data
        ball.x_pos.append(x)
        ball.y_pos.append(y)
        ball.z_pos.append(z)

        # Calculate the total velocity magnitude
        v_total = math.sqrt(v_x**2 + v_y**2 + v_z**2)

        # Drag forces (using v_total to calculate the drag magnitude but applying it directionally)
        x_force_due_to_drag = -0.5 * ball.cd * rho * ball.csa * v_total * v_x
        y_force_due_to_drag = -0.5 * ball.cd * rho * ball.csa * v_total * v_y
        z_force_due_to_drag = -0.5 * ball.cd * rho * ball.csa * v_total * v_z

        # Calculate the Magnus Forces
        ang_v_x = (math.pi/30) * ball.backspin
        ang_v_y = 0
        ang_v_z = (math.pi/30) * ball.sidespin
        f_mag_x = magnus_coefficient * ((ang_v_y*v_z) - (ang_v_z*v_y))
        f_mag_y = magnus_coefficient * ((ang_v_z*v_x) - (ang_v_x*v_z))
        f_mag_z = magnus_coefficient * ((ang_v_x*v_y) - (ang_v_y*v_x))

        # Force due to gravity (only affects the z-direction)
        force_due_to_gravity = ball.mass * a_grav

        # Sum of forces in each direction
        sum_of_x_forces = x_force_due_to_drag + f_mag_x
        sum_of_y_forces = y_force_due_to_drag + f_mag_y
        sum_of_z_forces = z_force_due_to_drag + force_due_to_gravity + f_mag_z

        # Update velocities based on the force applied and the mass of the projectile
        v_x += (sum_of_x_forces / ball.mass) * dt
        v_y += (sum_of_y_forces / ball.mass) * dt
        v_z += (sum_of_z_forces / ball.mass) * dt

        # Update positions based on the current velocities
        x += v_x * dt
        y += v_y * dt
        z += v_z * dt

        # Increment the timestep counter
        timestep_count += 1

        # Optional debugging output to track the ball's state
        print(f"Name: {ball.name}, Time: {round(timestep_count * dt, 4)}s, x: {round(x, 2)}, y: {round(y, 2)}, z: {z}")


# Creating the various golf balls - this example shows the effect of backspin on the flight path of a ball and distance travelled
ball1 = golf_ball(name="Ball_1", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=0, sidespin_rpm=0)
ball2 = golf_ball(name="Ball_2", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=1000, sidespin_rpm=0)
ball3 = golf_ball(name="Ball_3", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=2000, sidespin_rpm=0)
ball4 = golf_ball(name="Ball_4", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=3000, sidespin_rpm=0)
ball5 = golf_ball(name="Ball_5", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=4000, sidespin_rpm=0)
ball6 = golf_ball(name="Ball_6", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=5000, sidespin_rpm=0)
ball7 = golf_ball(name="Ball_7", launch_angle_degrees=12 , aim_angle_degrees=90 , launch_velocity=70 , backspin_rpm=6000, sidespin_rpm=0)


# List of balls - add as many to this list as you like
balls = [ball1, ball2, ball3 , ball4 , ball5 , ball6 , ball7]




# ----------------------- Hitting the golf balls, no required input from user ----------------------- #




# Simulate the shots
for ball in balls:
    hit_golf_ball(ball)


# Print a space in the terminal
print("""
      
#-----------------------------------------------------------------------------------------#

      """)


# Print the landing data
for ball in balls:
    print(str(ball))


# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for ball in balls:

    ax.plot(ball.x_pos , ball.y_pos , ball.z_pos , label=ball.name)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()

# Set the lower limit of the z-axis to 0

plt.axis('equal')
plt.show()