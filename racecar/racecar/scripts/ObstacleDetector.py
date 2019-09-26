import time

class ObstacleDetector:
    def __init__(self):
        self.mode = 0 # 0: Left, 1: Right
        self.previous_time = 0

    # If detected, return True and change mode
    def check(self, obstacles):
        if time.time() < self.previous_time + 1:
            return False
        for circle in obstacles.circles:
            p = circle.center
            if abs(p.x) < 0.3 and -1 < p.y < 0:
                self.previous_time = time.time()
                self.mode = 1 - self.mode
                return True
        return False

obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data

if __name__ == '__main__':
    from obstacle_detector.msg import Obstacles
    import rospy
    rospy.init_node('test')
    rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
    ob = ObstacleDetector()

    time.sleep(3)

    while not rospy.is_shutdown():
        ob.check(obstacles)
        print(ob.mode)
        time.sleep(0.1)

    print('Done')
