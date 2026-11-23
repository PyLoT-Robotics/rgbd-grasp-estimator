from glob import glob
from setuptools import setup,find_packages
setup(name='rgbd_grasp',version='0.1.0',description='RGB-D region clustering and geometric grasp candidates',python_requires='>=3.10',
      packages=find_packages(exclude=['test']),install_requires=['numpy'],extras_require={'test': ['pytest']},
      license='Apache-2.0',maintainer='PyLoT Robotics',maintainer_email='163533636+PyLoT-Robotics@users.noreply.github.com',
      data_files=[('share/ament_index/resource_index/packages',['resource/rgbd_grasp']),
                  ('share/rgbd_grasp',['package.xml']),('share/rgbd_grasp/config',glob('config/*')),
                  ('share/rgbd_grasp/launch',glob('launch/*.py'))],
      entry_points={'console_scripts':['rgbd-grasp-estimator = rgbd_grasp.cli:main','rgbd_grasp_node = rgbd_grasp.node:main']})
