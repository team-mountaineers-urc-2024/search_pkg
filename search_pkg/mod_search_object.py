#Zachary Spiggle 2/4/24
#An object to handle searching needs
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from mpl_toolkits.mplot3d import Axes3D


class Search_Object:

    # Setup Variables
    center = np.array([0, 0])   # Centerpoint of search circle
    radius = 15  # Radius of search circle
    standard_deviation = radius / 3    # Standard deviation of pdf distribution
    grid_count = 100  # Number of gridlines in X and Y direction of search area (resolution)
    observation_radius = 3   # Robot Observation radius
    observation_certainty = 1  # Robot Observation Certainty
    decay_percent = 0  # Probability decay when redistributing probability from observed area
    min_sample_dist = observation_radius * 1.5  # Minimum distance between sample search points
    robot_position = np.array([-20, 0])    # Robot start position
    object_located = 0

    # Sample points
    n_sample = 7  # Number of points to sample
    XY_samples = np.empty((2, 0))  # Matrix of coordinate indices of each sample
    l = 0  # Counter variable
    attempts = 0  # Attempts made to generate points, used to break while loop
    sample_dist = np.zeros(n_sample) + 1000  # Distance between samples, set to a large number initially
    
    #Define variables so that they will get assigned in Init
    theta = None
    x_circ = None
    y_circ = None
    dx = None
    x = None
    y = None
    covar = None
    X = None
    z = None
    goal_index = None
    goal_position = None
    n_sample = None
    ordered_samples = None
    next_position = None
    line_vector = None


    def __init__(self, robot_pos=[0,0], rad=15, sample_size=7 ):
        self.robot_position = np.array(robot_pos)
        self.center = np.array(robot_pos) #Want the center of the circle to form around the robot
        self.radius = rad 

        # Draw search circle
        self.theta = np.arange(0, 2.1 * np.pi, 0.1)
        self.x_circ = self.radius * np.cos(self.theta) + self.center[0]
        self.y_circ = self.radius * np.sin(self.theta) + self.center[1]

        # Discretize area in x and y direction
        self.dx = 2 * self.radius / self.grid_count
        self.x = np.linspace(self.center[0] - self.radius, self.center[0] + self.radius, self.grid_count)
        self.y = np.linspace(self.center[1] - self.radius, self.center[1] + self.radius, self.grid_count)

        # Create a two-row array that stores all of the coordinates in the grid
        self.X = np.zeros((2, self.grid_count**2))
        for i in range(len(self.x)):
            self.X[0, i*len(self.x):(i+1)*len(self.x)] = self.x[i]  # Row 1 contains x coords
            self.X[1, i*len(self.y):(i+1)*len(self.y)] = self.y      # Row 2 contains y coords

        self.covar = np.array([[self.standard_deviation**2, 0], [0, self.standard_deviation**2]])  # pdf covariance  # Probability distribution
        self.z = np.reshape(multivariate_normal.pdf(self.X.T, self.center, self.covar), (self.grid_count, self.grid_count))  # Probability distribution
        self.z = self.z.flatten(order='C')

        # Generate goal point
        self.goal_index = np.random.choice(np.arange(len(self.z)), 1, p=self.z / np.sum(self.z))[0]  # Chosen index of current sample
        self.goal_position = np.array([self.X[0, self.goal_index], self.X[1, self.goal_index]])

        self.n_sample = sample_size
        #print(self.z)

    #HELPER FUNCTIONS ---------------------
    #Salesman solution
    def shrinkwrap_salesman(self, points):
        points = points.T
        zeros_column = np.zeros((points.shape[0], 1))
        points = np.hstack((points, zeros_column))
        n = points.shape[0]
        centroid = [np.sum(points[0, :]) / n, np.sum(points[1, :]) / n]
        for i in range(n):
            points[i, 2] = np.arctan2(points[i, 1] - centroid[1], points[i, 0] - centroid[0])
            if points[i,2] < points[0, 2]:
                points[i,2] += 2 * np.pi     
        points_sorted = points[np.argsort(points[:, 2]),: ]
        ordered_points = points_sorted[:, :2]
        
        return ordered_points.T

    #Add intermediate measurements along a line - would be replaced by rover running realtime?
    def calculate_line_logic(self, X, robot_position, next_position, observation_radius, j, line_vector):
        # Calculate distance from point to a line drawn between the current
        # position and the next position being traveled to
        point_vector1 = np.array([X[0, j], X[1, j]]) - robot_position
        point_vector2 = np.array([X[0, j], X[1, j]]) - next_position
        point_projection = np.dot(point_vector1, line_vector) / np.linalg.norm(line_vector)**2
        closest_line_point = robot_position + point_projection * line_vector
        line_dist = np.linalg.norm(np.array([X[0, j], X[1, j]]) - closest_line_point)
        dot_product1 = np.dot(point_vector1, line_vector)
        dot_product2 = np.dot(point_vector2, line_vector)

        # Check if the point is between the two specified endpoints
        is_between_endpoints = (dot_product1 >= 0 and dot_product2 <= 0) or (dot_product1 <= 0 and dot_product2 >= 0)
        start_dist = np.sqrt((robot_position[0] - X[0, j])**2 + (robot_position[1] - X[1, j])**2)

        if line_dist <= observation_radius and is_between_endpoints and start_dist >= observation_radius:
            line_logic = 1
        else:
            line_logic = 0

        return line_logic
    

    #WAYPOINT GENERATION AND MANAGEMENT
    #Reset variables to handle a new search circle
    def reset_algrothim(self, center_pos=[0,0], rad=15, sample_size=7, observation_radius=3, observation_certainy=1, grid_count=100):
        # Setup Variables
        self.center = np.array(center_pos)   # Centerpoint of search circle
        self.radius = 15  # Radius of search circle
        self.standard_deviation = self.radius / 3    # Standard deviation of pdf distribution
        self.grid_count = grid_count  # Number of gridlines in X and Y direction of search area (resolution)
        self.observation_radius = observation_radius  # Robot Observation radius
        self.observation_certainty = observation_certainy  # Robot Observation Certainty
        self.decay_percent = 0  # Probability decay when redistributing probability from observed area
        self.min_sample_dist = self.observation_radius * 1.5  # Minimum distance between sample search points
        self.robot_position = np.array([-20, 0])    # Robot start position
        self.object_located = 0

        # Sample points
        self.n_sample = 7  # Number of points to sample
        self.XY_samples = np.empty((2, 0))  # Matrix of coordinate indices of each sample
        self.l = 0  # Counter variable
        self.attempts = 0  # Attempts made to generate points, used to break while loop
        self.sample_dist = np.zeros(self.n_sample) + 1000  # Distance between samples, set to a large number initially
        
        self.robot_position = np.array(center_pos) #THIS SHOULDNT MATTER
        self.center = np.array(center_pos) #Want the center of the circle to form around the robot
        self.radius = rad 

        # Draw search circle
        self.theta = np.arange(0, 2.1 * np.pi, 0.1)
        self.x_circ = self.radius * np.cos(self.theta) + self.center[0]
        self.y_circ = self.radius * np.sin(self.theta) + self.center[1]

        # Discretize area in x and y direction
        self.dx = 2 * self.radius / self.grid_count
        self.x = np.linspace(self.center[0] - self.radius, self.center[0] + self.radius, self.grid_count)
        self.y = np.linspace(self.center[1] - self.radius, self.center[1] + self.radius, self.grid_count)

        # Create a two-row array that stores all of the coordinates in the grid
        self.X = np.zeros((2, self.grid_count**2))
        for i in range(len(self.x)):
            self.X[0, i*len(self.x):(i+1)*len(self.x)] = self.x[i]  # Row 1 contains x coords
            self.X[1, i*len(self.y):(i+1)*len(self.y)] = self.y      # Row 2 contains y coords

        self.covar = np.array([[self.standard_deviation**2, 0], [0, self.standard_deviation**2]])  # pdf covariance
        #z = multivariate_normal.pdf(X.T, center, covar).reshape(-1, 1)  # Probability distribution
        self.z = np.reshape(multivariate_normal.pdf(self.X.T, self.center, self.covar), (self.grid_count, self.grid_count))  # Probability distribution
        self.z = self.z.flatten(order='C')

        # Generate goal point
        self.goal_index = np.random.choice(np.arange(len(self.z)), 1, p=self.z / np.sum(self.z))[0]  # Chosen index of current sample
        self.goal_position = np.array([self.X[0, self.goal_index], self.X[1, self.goal_index]])

        self.n_sample = sample_size

        self.display_init()


    #Generate points - (next mainloop run) - returns points to evaluate
    def generate_points(self):
        
        #RESET THESE VARIABLES EACH RUN
        self.XY_samples = np.empty((2, 0))  # Matrix of coordinate indices of each sample
        self.l = 0  # Counter variable
        self.attempts = 0  # Attempts made to generate points, used to break while loop
        self.ample_dist = np.zeros(self.n_sample) + 1000  # Distance between samples, set to a large number initially


        #Print statements dont work in ROS2
        #print(f'The sum of probabilities is {np.sum(self.z) * self.dx**2:.5f}') #Sum the probabilities
        #of the whole grid, should be close to one on first iteration, decreases based on decay %

        # Generate and evaluate samples
        while self.l < self.n_sample:
            self.l = self.XY_samples.shape[1]  # Counter of the number of samples selected

            self.sample_index = np.random.choice(np.arange(len(self.z)), 1, p=self.z / np.sum(self.z))  # Chosen index of the current sample
            self.sample_index = self.sample_index[0]

            if self.l == 0:
                # Add the first coordinates to the matrix
                self.XY_samples = np.array([[self.X[0, self.sample_index]], [self.X[1, self.sample_index]]])
            else:
                self.XY_potential = np.array([self.X[0, self.sample_index], self.X[1, self.sample_index]])
                self.sample_dist = np.zeros(self.l) + 1000  # Initializing sample distances with a large value

                for i in range(self.l):
                    # Calculate the distance between the current sample and all previous
                    # samples from this iteration
                    self.sample_dist[i] = np.sqrt((self.XY_potential[0] - self.XY_samples[0, i]) ** 2 + (self.XY_potential[1] - self.XY_samples[1, i]) ** 2)

                if np.min(self.sample_dist - self.min_sample_dist) > 0:
                    # Only add the sample position if it is farther than the specified
                    # minimum sample distance from all other samples
                    self.XY_samples = np.concatenate((self.XY_samples, self.XY_potential.reshape(-1, 1)), axis=1)
                else:
                    self.attempts += 1

            
            if self.attempts > 1000:
                raise ValueError('Attempts exceeded maximum allowance')


        #self.display_points()
        #self.update_display()

        #Put the robots current position as the first position in the algorithm's set of points
        #self.XY_samples = np.insert(self.XY_samples, 0, self.robot_position, axis=1)
        #print(self.XY_samples.T)
        self.ordered_samples = self.shrinkwrap_salesman(self.XY_samples)
        #self.ordered_samples = np.insert(self.ordered_samples, 0, self.robot_position, axis=1) #insert it after so that it is the first point
        #print(self.ordered_samples.T)
        return self.ordered_samples



    def redist_point(self, point):
        #Update PDF
        far_cell_count = 0 #Count of cells outside of observation radius
        total_removed = 0 #Sum of obervation probability removed
        
        for j in range(len(self.z)):
            #Calculate distance from robot of each cell
            point_dist = np.sqrt((point[0] - self.X[0, j])**2 + (point[1] - self.X[1, j])**2)

            #Calculate distance from centerpoint of each cell
            center_dist = np.sqrt((self.center[0] - self.X[0, j])**2 + (self.center[1] - self.X[1, j])**2)


            #if dist <= observation_radius and center_dist <= radius:
            if center_dist <= self.radius and (point_dist <= self.observation_radius):      
                    #line logic is a check if the tile is between the current position and the next position (falls in the line)
                    #how do we just search around rover as it navigates?

                #Probability removed from each cell
                removed_i = self.z[j] - self.z[j] * (1 - self.observation_certainty)
                total_removed += removed_i
                self.z[j] *= (1 - self.observation_certainty)


            elif center_dist <= self.radius:
                far_cell_count += 1

        distribution_per_cell = total_removed / far_cell_count * (1 - self.decay_percent)


        for k in range(len((self.z))):
            #Calculate distance from robot and center
            center_dist = np.sqrt((self.center[0] - self.X[0, k])**2 + (self.center[1] - self.X[1, k])**2)
            #line_logic = self.calculate_line_logic(self.X, self.robot_position, self.next_position, self.observation_radius, j, self.line_vector)

            if center_dist<=self.radius and not((point_dist<=self.observation_radius )):
                #Distribute removed probability to these cells
                self.z[k] += distribution_per_cell
        

        self.robot_position = point#self.next_position


    #Searched area - tell algorithm a point has been fine
    def redist_original(self):
        #Update PDF
        far_cell_count = 0 #Count of cells outside of observation radius
        total_removed = 0 #Sum of obervation probability removed
        
        for j in range(len(self.z)):
            #Calculate distance from robot of each cell
            end_dist = np.sqrt((self.next_position[0] - self.X[0, j])**2 + (self.next_position[1] - self.X[1, j])**2)

            #Calculate distance from centerpoint of each cell
            center_dist = np.sqrt((self.center[0] - self.X[0, j])**2 + (self.center[1] - self.X[1, j])**2)

            line_logic = self.calculate_line_logic(self.X, self.robot_position, self.next_position, self.observation_radius, j, self.line_vector)

            #If points are inside observation radius and total search radius

            #if dist <= observation_radius and center_dist <= radius:
            if center_dist <= self.radius and (line_logic or end_dist <= self.observation_radius):      
                    #line logic is a check if the tile is between the current position and the next position (falls in the line)
                    #how do we just search around rover as it navigates?

                #Probability removed from each cell
                removed_i = self.z[j] - self.z[j] * (1 - self.observation_certainty)
                total_removed += removed_i
                self.z[j] *= (1 - self.observation_certainty)


                #We arent checking for the object with the algorithm, just plotting the path

                #Actual check for finding 
                # self.object_located = bool
                
                # #Simulated Check (for example purposes)
                # if self.X[0, j] == self.goal_position[0] and self.X[1, j] == self.goal_position[1]:
                #     if np.random.rand() < self.observation_certainty:
                #         self.object_located = True


            elif center_dist <= self.radius:
                far_cell_count += 1

        distribution_per_cell = total_removed / far_cell_count * (1 - self.decay_percent)


        for k in range(len((self.z))):
            #Calculate distance from robot and center
            center_dist = np.sqrt((self.center[0] - self.X[0, k])**2 + (self.center[1] - self.X[1, k])**2)
            line_logic = self.calculate_line_logic(self.X, self.robot_position, self.next_position, self.observation_radius, j, self.line_vector)

            if center_dist<=self.radius and not((line_logic or end_dist<=self.observation_radius )):
                #Distribute removed probability to these cells
                self.z[k] += distribution_per_cell
        

        self.robot_position = self.next_position

    #Not good practice, should probably be updated
    # def update_positions(self, next_pos, line_vec):
    #     #self.ordered_samples = ordered_values
    #     self.next_position = next_pos
    #     self.line_vector = line_vec
    #     #self.robot_position = robot_pos


    #GRAPHING FUNCTIONS --------------
    #initialize the display (Does not show anything)
    def display_init(self):
        # Create a 1x3 subplot grid
        self.fig = plt.figure(figsize=(15, 5))
        self.ax1 = self.fig.add_subplot(131)

        # Plotting the circle
        self.ax1.plot(self.x_circ, self.y_circ, 'k-')

        # Plotting goal position
        # self.ax1.plot(self.goal_position[0], self.goal_position[1], 'g*', markersize=10)
        # self.ax1.set_title('Goal Position')
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.axis('equal')


        # Plotting 3D probability distribution
        self.ax2 = self.fig.add_subplot(132, projection='3d')
        self.scatter = self.ax2.scatter(self.X[0], self.X[1], self.z, c=self.z, cmap='viridis', marker='.')
        #self.ax2.plot(self.goal_position[0], self.goal_position[1], 'r*', markersize=10)
        self.ax2.set_title('3D Probability Distribution')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Probability')

        # Contour plot
        self.ax3 = self.fig.add_subplot(133)
        self.contour_plot = self.ax3.contourf(self.X[0].reshape((self.grid_count, self.grid_count)), self.X[1].reshape((self.grid_count, self.grid_count)), self.z.reshape((self.grid_count, self.grid_count)), cmap='viridis')
        self.ax3.plot(self.goal_position[0], self.goal_position[1], 'r*', markersize=10)
        self.ax3.set_title('2D Probability Distribution')
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        self.fig.colorbar(self.contour_plot, ax=self.ax3, label='Probability')

    def display_points(self):
        # Plot samples on figure   
        self.ax1.plot(self.XY_samples[0, :], self.XY_samples[1, :], 'rx')
        self.ax1.plot(self.robot_position[0], self.robot_position[1], 'bx')


    def draw_line(self, point1, point2):
        #Draw line
        self.ax1.plot(point1, point2, 'r-')
        plt.pause(0.1)

    def update_display(self):

        self.display_points()


        # Display color graph
        self.ax2.clear()
        self.ax2.plot_surface(self.X[0].reshape((self.grid_count, self.grid_count)), self.X[1].reshape((self.grid_count, self.grid_count)), self.z.reshape((self.grid_count, self.grid_count)), cmap='viridis')
        self.ax2.set_xlabel('X')
        self.ax2.set_ylabel('Y')
        self.ax2.set_zlabel('Probability')
        plt.pause(0.05)
        
        self.ax3.clear()
        self.ax3.contourf(self.X[0].reshape((self.grid_count, self.grid_count)), self.X[1].reshape((self.grid_count, self.grid_count)), self.z.reshape((self.grid_count, self.grid_count)), cmap='viridis')
        self.ax3.plot(self.goal_position[0], self.goal_position[1], 'r*', markersize=10) #Plot endpoint in probabilty distribution
        self.ax3.set_xlabel('X')
        self.ax3.set_ylabel('Y')
        #plt.draw()
        plt.pause(0.05)


    #Shows the algorithm
    def display_alg(self):
        plt.show()
        