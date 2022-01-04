from setuptools import find_packages, setup

setup(
    name="coffee",
    version="0.0.0",
    description="PyBullet infrastructure is great, and so is coffee.",
    author="Kevin Zakka",
    license="MIT",
    packages=find_packages(),
    package_data={"coffee": ["py.typed"]},
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "pybullet",
        "dm_robotics-transformations",
        "dm_robotics-geometry",
    ],
    extras_require={
        "testing": [
            "pytest",
            "absl-py",
        ],
        "examples": [
            "dcargs",
        ],
    },
)
