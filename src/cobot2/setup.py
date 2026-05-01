from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cobot2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[ 
    	('share/ament_index/resource_index/packages', ['resource/' + package_name]), 
    	('share/' + package_name, ['package.xml']), 
    	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 	 
    	(os.path.join('share', package_name, 'config'), glob('config/*.json')), # 추가: 모델 파일 설치  
    	(os.path.join('share', package_name, 'models'), glob('models/*')), # 추가: 환경 변수 및 데이터 파일  
    	(os.path.join('share', package_name), ['.env', 'cobot2/data.json']),
        (os.path.join('share', package_name), glob('cobot2/*.sh')),
        (os.path.join('lib', package_name), ['cobot2/run_voice.sh']), 
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='COBOT2 Chess Robot Control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stockfish = cobot2.stockfish:main',
            'robotaction = cobot2.robot_action:main',
            'main = cobot2.main:main',
            'object = cobot2.vision_db:main',
        ],
    },
)
