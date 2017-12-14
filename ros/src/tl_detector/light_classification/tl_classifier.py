from styx_msgs.msg import TrafficLight
from base_model import *
from tf_model import *
import rospy
class TLClassifier(object):
    def __init__(self):
        #self.model = BaseModel()
        self.model = TrafficLightModel()

    def get_classification(self, image, detection, is_site):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #start = time.time()
        pred = self.model.predict(image, detection, is_site)

        #end = time.time()
        #rospy.logwarn("Prediction time: {}".format(start-end))
        return pred
