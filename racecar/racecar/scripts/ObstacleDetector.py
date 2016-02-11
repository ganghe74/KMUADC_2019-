import time
import enum

class Position(enum.Enum):
    NO = 0
    YES = 1
    DANGER = 2

class ObstacleDetector:
    def __init__(self):
        self.mode = Position.NO
        self.previous_time = 0
        self.cnt = 0

    # return EnumClass Position
    def check(self, obstacles):
        for circle in obstacles.circles:
            p = circle.center
            if abs(p.x) < 0.3:
                if -1 < p.y < 0:
                    if p.y > -0.5:
                        self.mode = Position.DANGER
                    else:
                        self.mode = Position.YES
                    break
            else:
                self.mode = Position.NO
        return self.mode

obstacles = None

def obstacle_callback(data):
    global obstacles
    obstacles = data

if __name__ == '__main__':
    from obstacle_detector.msg import Obstacles
    import rospy
    rospy.init_node('TEST')
    rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
    ob = ObstacleDetector()

    time.sleep(3)

    while not rospy.is_shutdown():
        ob.check(obstacles)
        print(ob.mode)
        time.sleep(0.1)

    print('Done')
