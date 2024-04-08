from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
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
    # angle = angle*180/3.14
    
    new_angle = node.get_angle() + angle
    # new_angle = new_angle % 6.28
    # print(new_angle)
    # print(angle)
    # print("............................................")
    # time.sleep(0.1)

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


def a_star_implementation(first_point, end_point, first_orentation):
    global cmans_vel
    """
    Implements the A* algorithm to find the optimal path from the start point to the goal point.

    Parameters:
    - first_point (tuple): The (x, y) coordinates of the start point.
    - end_point (tuple): The (x, y) coordinates of the goal point.
    - first_orentation (int): The orientation of the robot at the start point.
    """
    # Create a priority queue to store the nodes to be explored
    list = PriorityQueue()
    # print(list)

    # Create a dictionary to store the visited nodes
    visited = dict()
    

    # Create a set to store the fixed nodes
    fixed_list = set()

    # Create the first node with the start point, orientation, and initial cost
    first_node = PossibleNode(first_point[0], first_point[1], first_orentation, None, 0, distance(first_point, end_point))

    # Add the first node to the priority queue, visited dictionary, and fixed list
    list.put((first_node.get_total_cost(), first_node))
    visited[(first_point[0], first_point[1])] = first_node
    fixed_list.add(first_point)
    count = 0
    while not list.empty():
        count += 1
        # Get the node with the lowest cost from the priority queue
        this_node = list.get()[1]
        this_point = (this_node.x, this_node.y)
        # print(this_point)
        goal_x, goal_y = end_point[0], end_point[1]
        # print(goal_x, goal_y)
        
        # Check if the current node is close to the goal point
        if math.sqrt((this_point[0] - goal_x) ** 2 + (this_point[1] - goal_y) ** 2) <= 100:
            # Perform backtracking to find the optimal path
            print("Goal Reached")
            backtrackPath = []
            vel = []
            ang = []
            while this_node is not None:
                backtrackPath.append((this_node.x , this_node.y, this_node.get_angle()))
                # frame[node.y, node.x] = (173, 46, 0)
                this_node = this_node.get_parent_node()
            backtrackPath.reverse()
            # print(backtrackPath)
            an =0

            for i in range(len(backtrackPath)-1):
                cv2.arrowedLine(frame, (backtrackPath[i][0], backtrackPath[i][1]), (backtrackPath[i + 1][0], backtrackPath[i + 1][1]), (0, 0, 255), 10, tipLength=0.1)
                x_velocity = (backtrackPath[i+1][0] - backtrackPath[i][0])/3
                y_velocity = (backtrackPath[i+1][1] - backtrackPath[i][1])/3
                velocity = math.sqrt(x_velocity**2 + y_velocity**2)/1000
                # angular_velocity = (backtrackPath[i+1][2] - backtrackPath[i][2])/dt
                angular_velocity = (math.atan(y_velocity/x_velocity)-an)/3
                an = math.atan(y_velocity/x_velocity)
                vel.append(0)
                vel.append(velocity)
                ang.append(-1*angular_velocity)
                ang.append(0)
                # print("Velocity: ", velocity)
                # print("Angular Velocity: ", angular_velocity)
                # # time.sleep(dt)
                frame_resized = cv2.resize(frame, (screen_width, screen_height))
                cv2.imshow("frame1",frame_resized)
                cv2.waitKey(10)
                # print(list)
                # writer_video.write(frame)
            cv2.waitKey(5000)
            return vel , ang

            
        action_list = [[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
        # print(action_list)
       
        for action in action_list:
            this_node_ = this_node
            x = 1
            while x>0.1:
                possible_next_node = move(this_node_, action)
               
                #    print(possible_next_node)
                if possible_next_node is not None:
                            this_node = possible_next_node
                            new_coordinate = possible_next_node.get_points()
                            # print(new_coordinate)
                            if new_coordinate not in fixed_list:
                                if new_coordinate not in visited:
                                    # Add the possible next node to the priority queue, visited dictionary, and fixed list
                                    list.put((possible_next_node.get_total_cost(), possible_next_node))
                                    visited[new_coordinate] = possible_next_node
                                    fixed_list.add(new_coordinate)
                                    
                                    if possible_next_node.get_parent_node() is not None:
                                        cv2.line(frame, possible_next_node.get_points(), possible_next_node.get_parent_node().get_points(), (255, 0, 0), 5)
                                        if count %100 == 0:
                                            writer_video.write(frame)
                                            frame_resized = cv2.resize(frame, (screen_width, screen_height))
                                            cv2.imshow("frame1",frame_resized)
                                            cv2.waitKey(10)
                                            writer_video.write(frame_resized)
                                        
                                            # print(list)
                                    
                                        
                                else:
                                    if visited[new_coordinate].get_total_cost() > possible_next_node.get_total_cost():
                                        # Update the visited node with a lower cost and add it to the priority queue
                                        visited[new_coordinate] = possible_next_node
                                        list.put((possible_next_node.get_total_cost(), possible_next_node))
                            
  
                x= x-0.1





writer_video = cv2.VideoWriter('GD.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 1000, (1920,1080))

if __name__ == "__main__":
    start_point, end_point, angle, goal_angle = inital_final_goals()
    vel,ang = a_star_implementation(start_point, end_point, angle)
    writer_video.release()
