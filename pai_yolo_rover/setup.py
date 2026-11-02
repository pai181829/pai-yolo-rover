import os
from glob import glob

from setuptools import setup

package_name = 'pai_yolo_rover'


def tree(src_dir, dst_prefix):
    """Collect (install_dir, [files]) pairs for a directory tree (used for Gazebo models)."""
    out = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        rel = os.path.relpath(root, src_dir)
        dst = os.path.join(dst_prefix, rel) if rel != '.' else dst_prefix
        out.append((dst, [os.path.join(root, f) for f in files]))
    return out


share = os.path.join('share', package_name)

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (os.path.join(share, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(share, 'urdf'), glob('urdf/*')),
        (os.path.join(share, 'worlds'), glob('worlds/*')),
        (os.path.join(share, 'config'), glob('config/*')),
    ] + tree('models', os.path.join(share, 'models')),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pai-student',
    maintainer_email='noreply@example.com',
    description='YOLO-guided object search rover for Gazebo (ROS 2 Humble)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_search = pai_yolo_rover.yolo_search_node:main',
            'demo_recorder = pai_yolo_rover.demo_recorder:main',
        ],
    },
)
