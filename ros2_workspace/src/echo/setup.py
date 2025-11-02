import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'echo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/all_nodes.launch.py']),
        (os.path.join('share', package_name, 'sounds'), glob(os.path.join(package_name, 'sounds', '*.wav'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven',
    maintainer_email='steven@vangemert.dev',
    description='Interactive voice chat system',
    license='AGPL-3.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stt_onboard = echo.stt_onboard:main',
            'tts_onboard = echo.tts_onboard:main',
            'initialization = echo.initialization:main',
            'sound_player = echo.sound_player:main',
            'speech_ai = echo.speech_ai:main',
        ],
    },
)
