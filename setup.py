from setuptools import find_packages, setup

setup(
    name="coffee",
    version="0.0.0",
    description="",
    author="Kevin Zakka",
    license="MIT",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "pybullet",
        "absl-py",
        "pytest",
        "dm_robotics-transformations",
        "dm_robotics-geometry",
        "dcargs",
    ],
)
