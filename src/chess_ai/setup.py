from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chess_ai'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'train_pt'), glob('chess_ai/train_pt/*.pt')),
        (os.path.join('share', package_name), ['.env', 'chess_ai/data.json']),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Chess AI Robot Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stockfish = chess_ai.stockfish:main',
            'robotaction = chess_ai.robot_action:main',
            'main = chess_ai.main:main',
            'object = chess_ai.vision_db:main',
            'gamelogger = chess_ai.game_logger:main',
        ],
    },
)
