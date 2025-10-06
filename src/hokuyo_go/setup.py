from setuptools import setup

setup(
    name='hokuyo_go',
    version='0.0.1',
    packages=['hokuyo_go'],
    install_requires=['rospy', 'sensor_msgs'],
    scripts=['scripts/depth_segmentation_node.py'],
    author='İsmail Eren Küçükali',
    author_email='ismailerenkucukali@gmail.com',
    description='Description of your package',
    license='MIT License',
    keywords='ROS',
)
