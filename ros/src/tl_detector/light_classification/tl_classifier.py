from styx_msgs.msg import TrafficLight
from base_model import *

class TLClassifier(object):
    def __init__(self):
        self.model = BaseModel()
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        pred = self.model.predict(image)

        return pred
