from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
import json
import cv2
from queue import PriorityQueue
from matplotlib import pyplot as plt



class PossibleNode :
    def __init__ (self, x , y, angle,  parent, g, h,action = None,v = None):
        """
        Represents a possible node in the A* algorithm.

        Parameters:
        - x (int): The x-coordinate of the node.
        - y (int): The y-coordinate of the node.
        - angle (int): The angle of the node.
        - parent (PossibleNode): The parent node.
        - g (float): The cost from the start node to this node.
        - h (float): The estimated cost from this node to the goal node.
        """
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = parent
        self.g = g
        self.h = h
        self.f = g + h
        self.action = action
        self.v = v

    def get_angle(self):
        """
        Returns the angle of the node.
        """
        return self.angle

    def get_points(self):
        """
        Returns the (x, y) coordinates of the node.
        """
        return (self.x , self.y)
    
    def get_total_cost(self):
        """
        Returns the total cost of the node (g + h).
        """
        return self.g + self.h
    
    def get_parent_node(self):
        """
        Returns the parent node.
        """
        return self.parent
    
    def __hash__ (self):
        """
        Returns the hash value of the node.
        """
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        """
        Checks if two nodes are equal based on their coordinates.
        """
        return self.x == other.x and self.y == other.y 
    
    def __lt__(self, other):
        """
        Compares two nodes based on their total cost.
        """
        return self.f < other.f
    # def__action__(self):
    #     return self.action
    

frame = np.zeros((2000, 6000, 3), dtype=np.uint8 )
y = 2000
initial_orientation = None

# robotClearance = 300
robotClearance = int(input("input robotClearance: "))
cv2.rectangle(frame, (1500, y-2000), (1750, y-1000), (255, 0, 255), (robotClearance))
cv2.rectangle(frame, (1500, y-2000), (1750, y-1000), (0, 255, 255), -1)
cv2.rectangle(frame, (2500, y-1000), (2750, y), (255, 0, 255), robotClearance)
cv2.rectangle(frame, (2500, y-1000), (2750, y), (0, 255, 255), -1)
cv2. circle(frame, (4200, y-1200), 600, (255, 0, 255), robotClearance)
cv2. circle(frame, (4200, y-1200), 600, (0, 255, 255), -1)

screen_width = 1200 # adjust to your screen resolution
screen_height = 400  # adjust to your screen resolution
frame_resized = cv2.resize(frame, (screen_width, screen_height))


def could_move(cord):
    """
    Checks if a given coordinate is a valid move.

    Parameters:
    - cord (tuple): The (x, y) coordinates to check.

    Returns:
    - bool: True if the coordinate is a valid move, False otherwise.
    """
    if cord[0] < (robotClearance+1) or cord[0] > 6000-(robotClearance+1) or cord[1] < (robotClearance+1) or cord[1] > 2000-(robotClearance+1):
        return False 
    cord_color = frame[cord[1], cord[0]]
    if cord_color[0] == 0 and cord_color[1] == 0 and cord_color[2] == 0:
        return True
    # if cord_color[0] == 220 and cord_color[1] == 180 and cord_color[2] == 120:z
    #     return True
    return False
def inital_final_goals():
    """
    Prompts the user to enter the initial and goal points, along with orientations.

    Returns:
    - initial_point (tuple): The (x, y) coordinates of the initial point.
    - goal_point (tuple): The (x, y) coordinates of the goal point.
    - initial_orientation (int): The orientation of the robot at the initial point.
    - goal_orientation (int): The orientation of the robot at the goal point.
    """
    global robotClearance
    global robot_stepSize
    global initial_orientation
    
    while True:
        x_coordinate, y_coordinate, initial_orientation = map(int, input("Enter the x and y coordinates of the initial point, followed by the orientation of the robot, separated by spaces: ").split())
        initial_point = (x_coordinate, 2000-y_coordinate)

        x_coordinate_goal, y_coordinate_goal = map(int, input("Enter the x and y coordinates of the goal point, of the robot separated by spaces: ").split()) 
       
        # robot_stepSize = int(input("step size is : "))
        goal_point = (x_coordinate_goal, 2000- y_coordinate_goal)

        goal_orientation = 0
        initial_orientation = math.radians(initial_orientation)
        

        if could_move(initial_point) and could_move(goal_point):
            return initial_point, goal_point , initial_orientation, goal_orientation
t = 0.1
r = 30
L = 354
dt = 0.5
angle = 0
# print()

def move(node, action):
    # print(node.v)
    global angle
    print("............................................")
    # angle = 0
    angle = (r / L) * (action[0] - action[1]) * dt
    
    new_angle = node.get_angle() + angle
    dx = r/2*(action[0]+action[1])*math.cos((new_angle))*dt 
    dy = r/2*(action[0]+action[1])*math.sin((new_angle))*dt
    vx = r/2*(action[0]+action[1])*math.cos(math.radians(new_angle))
    vy = r/2*(action[0]+action[1])*math.sin(math.radians(new_angle))
    v= math.sqrt(vx**2 + vy**2)
    new_x = node.x + round(dx)
    new_y = node.y + round(dy)
    g = math.sqrt(dx**2 + dy**2)
    if not could_move((new_x, new_y)):
        return None
    return PossibleNode(new_x, new_y, new_angle, parent=node, g=node.g+g , h=distance((new_x,new_y), end_point),action= action,v=v)

        
def distance(pt1, pt2):
    """
    Calculates the Euclidean distance between two points.

    Parameters:
    - pt1 (tuple): The (x, y) coordinates of the first point.
    - pt2 (tuple): The (x, y) coordinates of the second point.

    Returns:
    - float: The Euclidean distance between the two points.
    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

rpm1 = float(input("\nEnter first RPM for the wheel: \n"))
rpm2 = float(input("\nEnter second RPM for the wheel: \n"))
# rpm1 = 60
# rpm2 = 30
rpm1= rpm1 * ((2*np.pi)/60)
rpm2 = rpm2 * ((2*np.pi)/60)
action_list = [[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]






writer_video = cv2.VideoWriter('GD.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 1000, (1920,1080))

if __name__ == "__main__":
    start_point, end_point, angle, goal_angle = inital_final_goals()
    writer_video.release()
