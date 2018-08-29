from jsk_hrp2_ros_bridge.jsk_hrp2_dashboard import *
import std_msgs.msg
import rospkg
hrpsys_gazebo_tutorials_path=rospkg.RosPack().get_path("hrpsys_gazebo_tutorials")

class CraneButtonMenu(MenuDashWidget):
    def __init__(self):
        icons = [['bg-grey.svg', 'ic-runstop-off.svg'],
                 ['bg-green.svg', 'ic-runstop-on.svg'],
                 ['bg-red.svg', 'ic-runstop-off.svg']]
        super(CraneButtonMenu, self).__init__('set Crane', icons)
        self.update_state(0)
        self.add_action('Crane up', self.on_up)
        self.add_action('Crane down', self.on_down)
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
        self.lift_command_pub = rospy.Publisher("/HRP2JSKNTS/CranePlugin/LiftCommand", std_msgs.msg.Empty, queue_size = 1)
        self.lower_command_pub = rospy.Publisher("/HRP2JSKNTS/CranePlugin/LowerCommand", std_msgs.msg.Empty, queue_size = 1)
    def on_up(self):
        self.lift_command_pub.publish(std_msgs.msg.Empty())
    def on_down(self):
        self.lower_command_pub.publish(std_msgs.msg.Empty())

class JSKHRP2gazeboDashboard(JSKHRP2Dashboard):
    def setup(self, context):
        super(JSKHRP2gazeboDashboard,self).setup(context)
        self.name = "JSK hrp2 gazebo dashboard"
        rospy.set_param("hrpsys_config_path", hrpsys_gazebo_tutorials_path+"/scripts")
        rospy.set_param("hrpsys_config_script_name", "jsk_hrp2_gazebo_setup")
        rospy.set_param("hrpsys_nshost", "localhost")
        self._crane_button = CraneButtonMenu()
    def get_widgets(self):
        ret = super(JSKHRP2gazeboDashboard,self).get_widgets()
        ret[-2].append(self._crane_button)
        return ret
