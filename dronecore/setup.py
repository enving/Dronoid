from setuptools import find_packages, setup

package_name = 'dronecore'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'pydantic',
        'httpx',
    ],
    zip_safe=True,
    maintainer='Claude & Tristan',
    maintainer_email='noreply@anthropic.com',
    description='Universal Drone OS - Natural Language Control for PX4 Drones',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_commander = dronecore.drone_commander:main',
            'nl_bridge = dronecore.nl_bridge:main',
        ],
    },
)
