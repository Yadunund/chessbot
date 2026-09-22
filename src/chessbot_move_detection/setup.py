from setuptools import find_packages, setup

package_name = "chessbot_move_detection"

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
    description="Move detection for chessbot: which move the human played.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "classic_detector = chessbot_move_detection.classic_detector:main",
            "gemma_detector = chessbot_move_detection.gemma_detector:main",
        ]
    },
)
