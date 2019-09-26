import time

class ObstacleDetector:
    def __init__(self):
        self.mode = 0 # 0: Left, 1: Right
        self.previous_time = 0

    def Check(self, obstacles):
        if time.time() < previous_time + 1:
            return False
        for circle in obstacles.circles:
            p = circle.center
            if p.x < abs(0.3) and 0 < p.y < 1:
                mode = 1 - mode
                return True
        return False