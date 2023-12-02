import rospy
from rover.msg import TransformMsg


class RoverLocationSate:

    #currentX - current X location (relative to start point, starts at 0,0)
    #currentY - current Y location (relative to start point, starts at 0,0)
    #direction - 0,1,2,3 -> 4 directions, 0 is the start direction
    #visited - visited locations stored as tuple in list
    #path - order of locations visited by rover

    def __init__(self):
        self.currentX = 0
        self.currentY = 0
        self.direction = 0
        self.visited = []
        self.path = []

    def get_location(self):
        return (self.currentX, self.currentY)

    # gets the x,y location if we move forward in direction 0
    def get_next_loc_direction0(self):
        return (self.currentX+1, self.currentY)

    # gets the x,y location if we move forward in direction 1
    def get_next_loc_direction1(self):
        return (self.currentX, self.currentY-1)

    # gets the x,y location if we move forward in direction 2
    def get_next_loc_direction2(self):
        return (self.currentX-1, self.currentY)

    # gets the x,y location if we move forward in direction 3
    def get_next_loc_direction3(self):
        return (self.currentX, self.currentY+1)


class ArucoGridSearchNode():


    #obstacle_node_name - name of the node that detects obtsacles
    #rover_loc_state - the location state of the rover
    #DIST_MOVE_DEFAULT_METERS - the default distance in meters to go forward if no obstacle detected

    def __init__(self, obstacle_node_name):
        self.obstacle_node_name = obstacle_node_name
        self.rover_loc_state = RoverLocationSate()
        self.rover_loc_state.visited.append(self.rover_loc_state.getLocation())
        self.DIST_MOVE_DEFAULT_METERS = 5

    #publishes the rotation angle and distance to move forward
    def publish_transformation(self, rotation, forward):
        pub = rospy.Publisher('aruco_search_node', TransformMsg, queue_size=10)

        transform = TransformMsg()
        transform.rotation_angle = rotation
        transform.travel_distance = forward

        if not rospy.is_shutdown():
            ropsy.loginfo(transform)
            pub.publish(msg)
    
    # callback for obstacle detection node
    def obstacle_callback(self, data):
        self.obstacle_detected = (bool)(data.data)
        self.move()

    # callback for ar tag detection node
    def ar_tag_detected_callback(self, data):
        self.ar_tag_detected = (bool) (data.data)

    # returns if direction 0 has no obstacle and is not visited, if not, signal to move forward in this direction, and update state
    def check_direction0(self):
        self.publish_transformation(90*(self.rover_loc_state.direction-0), 0)
        self.rover_loc_state.direction = 0

        if not self.obstacle_detected and self.rover_loc_state.get_next_loc_direction0() not in visited and not self.ar_tag_detected():
            self.rover_loc_state.path.append(self.rover_loc_state.getLocation())
            self.rover_loc_state.currentX += 1
            self.publish_transformation(0, self.DIST_MOVE_DEFAULT_METERS)
            return True

        return False

    # returns if direction 1 has no obstacle and is not visited, signal to move forward in this direction, and update state
    def check_direction1(self):
        self.publish_transformation(90*(self.rover_loc_state.direction-1), 0)
        self.rover_loc_state.direction = 1

        if not self.obstacle_detected and self.rover_loc_state.get_next_loc_direction1() not in visited and not self.ar_tag_detected():
            self.rover_loc_state.path.append(self.rover_loc_state.getLocation())
            self.rover_loc_state.currentY -= 1
            self.publish_transformation(0, self.DIST_MOVE_DEFAULT_METERS)
            return True

        return False

    # returns if direction 2 has no obstacle and is not visited, if not, signal to move forward in this direction, and update state
    def check_direction2(self):
        self.publish_transformation(90*(self.rover_loc_state.direction-2), 0)
        self.rover_loc_state.direction = 2

        if not self.obstacle_detected and self.rover_loc_state.get_next_loc_direction2() not in visited and not self.ar_tag_detected():
            self.rover_loc_state.path.append(self.rover_loc_state.getLocation())
            self.rover_loc_state.currentX -= 1
            self.publish_transformation(0, self.DIST_MOVE_DEFAULT_METERS)
            return True

        return False

    # returns if direction 3 has no obstacle and is not visited, if not, signal to move forward in this direction, and update state
    def check_direction3(self):
        self.publish_transformation(90*(self.rover_loc_state.direction-3), 0)
        self.rover_loc_state.direction = 3

        if not self.obstacle_detected and self.rover_loc_state.get_next_loc_direction3() not in visited and not self.ar_tag_detected():
            self.rover_loc_state.path.append(self.rover_loc_state.getLocation())
            self.rover_loc_state.currentY += 1
            self.publish_transformation(0, self.DIST_MOVE_DEFAULT_METERS)
            return True

        return False


    #checks each direction for obstacles and if they were visited
    #if all directions are visited/obstacles, then backtrack to previous location in the path
    # rotate and move as per node publish
    def move(self):
        self.rover_loc_state.visited.append(self.rover_loc_state.get_location())

        if self.check_direction0():
            return
        elif self.check_direction1():
            return
        elif self.check_direction2():
            return
        elif self.check_direction3():
            return
        else:
            prev_location = self.rover_loc_state.path.pop()
            target_direction = 0

            if prev_location[1] - self.rover_loc_state.currentY == -1:
                target_direction = 1
            elif prev_location[0] - self.rover_loc_state.currentX == -1:
                target_direction = 2
            else:
                target_direction = 3
            
            self.publish_transformation(90*(self.rover_loc_state.direction - target_direction), self.DIST_MOVE_DEFAULT_METERS)


    def listener(self):
        rospy.init_node('aruco_search_node', anonymous=True)
        rospy.Subscriber(self.obstacle_node_name, self.obstacle_callback)
        rospy.Subscriber('aruco_tag_detector', self.ar_tag_detected)

        rospy.spin()

if __name__ == '__main__':
    aruco_search_node = ArucoGridSearchNode()
    aruco_search_node.listener()




