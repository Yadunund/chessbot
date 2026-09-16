from setuptools import find_packages, setup

package_name = "chessbot_brain"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yadunund Vijay",
    maintainer_email="yadunund@gmail.com",
    description="Game engine and orchestrator for chessbot.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["brain_node = chessbot_brain.brain_node:main"]},
)
