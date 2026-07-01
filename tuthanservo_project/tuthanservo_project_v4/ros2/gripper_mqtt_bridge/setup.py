from setuptools import find_packages, setup


package_name = "gripper_mqtt_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),
        (
            "share/" + package_name,
            ["package.xml"]
        ),
    ],
    install_requires=[
        "setuptools",
        "paho-mqtt>=2.1,<3"
    ],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="maintainer@example.com",
    description="ROS 2 to MQTT bridge for the Pico W gripper.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gripper_bridge = "
            "gripper_mqtt_bridge.node:main",
        ],
    },
)
