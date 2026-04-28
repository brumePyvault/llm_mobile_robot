from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'llm_mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', '.env.example']),
        (os.path.join('share', 'llm_mobile_robot', 'launch'),
        glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'python-dotenv', 'openai'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'llm_control = llm_mobile_robot.llm_control:main'
        ],
    },
)
