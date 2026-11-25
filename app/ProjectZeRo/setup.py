from setuptools import setup, find_packages

setup(
    name="project_zero_sdk",
    version="0.1.0",
    description="Modular Zenoh Telemetry, Time-Sync, and ROS 2 SDK for Project ZeRo",
    packages=find_packages(include=["project_zero_sdk*"]),
    python_requires=">=3.10",
    install_requires=[
        "eclipse-zenoh",
    ],
)
