import numpy as np
from styx_msgs.msg import TrafficLight

class BaseModel():
    """A simple baseline traffic light classifier.
    :param thresh: Minimum amount of pixel within a given color.
    :param max_color: Pixel color value to exceed to get counted.
    """
    def __init__(self):
        self.BLUE_CHANNEL = 0
        self.GREEN_CHANNEL = 1
        self.RED_CHANNEL = 2
        self.STATUS = ['RED', 'YELLOW', 'GREEN', '', 'UNKNOWN']
        self.thresh = 350
        self.max_color = 245

    def predict(self, image):
        pred = TrafficLight.UNKNOWN

        img_red = image[:,:,self.RED_CHANNEL]
        img_green = image[:,:,self.GREEN_CHANNEL]

        n_red = np.sum(self.max_color < img_red)
        n_green = np.sum(self.max_color < img_green)

        if n_green > n_red and n_green > self.thresh:
            pred = TrafficLight.GREEN
        elif n_red > n_green and n_red > self.thresh:
            pred = TrafficLight.RED

        return pred
