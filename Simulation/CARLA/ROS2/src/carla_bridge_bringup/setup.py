from setuptools import setup
import os
from glob import glob

package_name = 'carla_bridge_bringup'

setup_dir = os.path.dirname(os.path.abspath(__file__))
scenarios_root = os.path.abspath(os.path.join(setup_dir, '..', '..', 'scenarios'))
scenario_files = []
if os.path.isdir(scenarios_root):
    top_files = sorted(glob(os.path.join(scenarios_root, '*.py')))
    if top_files:
        scenario_files.append((os.path.join('share', package_name, 'scenarios'), [os.path.relpath(f, setup_dir) for f in top_files]))
    for map_name in sorted(os.listdir(scenarios_root)):
        map_dir = os.path.join(scenarios_root, map_name)
        if os.path.isdir(map_dir) and not map_name.startswith('__'):
            files = sorted(glob(os.path.join(map_dir, '*.py')))
            if files:
                rel_files = [os.path.relpath(f, setup_dir) for f in files]
                scenario_files.append((os.path.join('share', package_name, 'scenarios', map_name), rel_files))

setup(
    name=package_name,
    version='1.1.0',
    packages=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ] + scenario_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Atanasko Boris Mitrev',
    maintainer_email='atanasko.mitrev@autoware.org',
    description='Vision Pilot bringup launch files',
    license='Apache License 2.0',
)
