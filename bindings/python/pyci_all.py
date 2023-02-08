from cartesian_interface import pyci
from cartesian_interface import roscpp_utils as roscpp
from cartesian_interface import pyest
from cartesian_interface.affine3 import Affine3
from cartesian_interface.impedance import Impedance


def get_xbot_config(is_floating_base=True, prefix='', model_type='RBDL', urdf=None, srdf=None, framework='ROS'):
    import rospy
    from xbot_interface import config_options as co

    if urdf is None:
        urdf = rospy.get_param(prefix + 'robot_description')

    if srdf is None:
        srdf = rospy.get_param(prefix + 'robot_description_semantic')

    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', model_type)
    cfg.set_bool_parameter('is_model_floating_base', is_floating_base)

    cfg.set_string_parameter('framework', framework)

    return cfg

