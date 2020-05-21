

class Node():

    '''
        Node for Astar
    '''

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.distance = np.Inf
        self.hueristic = np.Inf
        self.parent = None

class PathPlanner():

    '''
        Path planner class
        Args:
            start point, goal point

    '''
    def __init__(self, start, goal, obstacles):


        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.nodes = [self.start, self.goal]
        self.path = []
    
    def make_nodes(self, resolution = 1):
        
        '''
            make nodes in the specified area
            Args:
                area = [length, width]
                resolution = number of nodes in 1m

            call this function everytime the path planner is called
        '''

        fig = plt.figure()
        self.ax = fig.add_subplot(111)

        CENTER = ((self.start.x + self.goal.x)/2, (self.start.y + self.goal.y)/2)
        dist = np.sqrt((self.start.x - self.goal.x)**2 + (self.start.y - self.goal.y)**2) / 2
        SEMI_AXES = (dist, dist * 1.5)
        ANGLE = np.degrees(np.arctan2(self.goal.x - self.start.x, self.goal.y - self.start.y))
        safety1r = 2
        safety2r = 2

        circle = Point(CENTER).buffer(1)
        ell = scale(circle, SEMI_AXES[0], SEMI_AXES[1])
        ell = rotate(ell, 90 - ANGLE)
        safety1 = Point((self.start.x, self.start.y)).buffer(safety1r)
        safety2 = Point((self.goal.x, self.goal.y)).buffer(safety2r)
        structure = cascaded_union([ell, safety1, safety2])

        for i in np.arange(-5 - safety1r , int(SEMI_AXES[0]*2 + safety2r + 5), resolution):
            for j in np.arange(-5 - int(SEMI_AXES[1]),int(SEMI_AXES[1] * np.cos(90 - ANGLE) + CENTER[1]) + 5):
                if Point(i, j).within(structure):
                    node = Node(i, j)
                    self.nodes.append(node)
                    self.ax.scatter(node.x, node.y, color='red', alpha=0.5)
                else:
                    continue
        
        ## check for the ellipse
        
        
        patch = PolygonPatch(structure, alpha = 0.5, color = 'green')
        self.ax.add_patch(patch)

    def transform_nodes(self, nodes):
        '''
            use this function to transform nodes to world frame
        '''
        pass

    def plan_path(self):
        self.start.distance = 0
        self.start.hueristic = self.start.distance + np.sqrt((self.start.x - self.goal.x)**2 + (self.start.y - self.goal.y)**2)
        while len(self.nodes) !=0:
            self.nodes = sorted(self.nodes, key=lambda node: node.hueristic)
            current = self.nodes[0]

            adjacent_nodes = [node for node in self.nodes if int(np.sqrt((node.x - current.x)**2 + (node.y - current.y)**2)) == 1]
            self.nodes.remove(current)
            
            if current.x == self.goal.x and current.y == self.goal.y:
                self.last = current
                print("---path planned---")
                break

            else:
                for node in adjacent_nodes:
                    if self.intersection(current, node) == True:
                        continue
                    else:
                        if node.distance > current.distance + np.sqrt((current.x - node.x)**2 + (current.y - node.y)**2):
                            
                            node.distance = current.distance + np.sqrt((current.x - node.x)**2 + (current.y - node.y)**2)
                            node.hueristic = node.distance + np.sqrt((self.goal.x - node.x) **2 + (self.goal.y - node.y) ** 2)
                            node.parent = current
                            

                            if node not in self.path:
                                self.path.append(node)


    def get_path(self):
        path = [self.goal]
        current = self.goal
        while current.parent != None:
            
            path.append(current.parent)
            current = current.parent
            
        return path

    def intersection(self, point1, point2):
        
        for obstacle in self.obstacles:
            if LineString([(p[0], p[1]) for p in obstacle]).intersects(LineString([(point1.x, point1.y), (point2.x, point2.y)])):
                return True
        
        else:
            return False
        
    def main(self, resolution = 1):   

        return path

    
if __name__ == '__main__':
    '''rospy.init_node('obst_detector')
    rate = rospy.Rate(10)
    
    detector = ObjectDetector()'''

    start = Node(0, 0)
    goal = Node(10, 10)

    obstacles = []

    for i in range(50):
        point = (random.random() * 10, random.random() * 10)
        obstacles.append([(point[0] + 0.5, point[1] + 0.5), (point[0] + 0.5, point[1] - 0.5), (point[0] - 0.5, point[1] - 0.5), (point[0] - 0.5, point[1] + 0.5)])
    
    planner = PathPlanner(start, goal, obstacles)
    planner.make_nodes(0.5)
    planner.plan_path()
    path = planner.get_path()
    planner.transform_nodes(path)

    for obstacle in obstacles:
        planner.ax.plot([p[0] for p in obstacle], [p[1] for p in obstacle], color = 'black')
    
    
    planner.ax.plot([p.x for p in path], [p.y for p in path], color='yellow')

    planner.ax.scatter([start.x, goal.x], [start.y, goal.y], color = 'green')
    plt.show()
