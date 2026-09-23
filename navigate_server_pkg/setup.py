from setuptools import find_packages, setup

package_name = "navigate_server"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="giu",
    maintainer_email="giu@todo.todo",
    description=(
        "GEORG Navigate.action server"
    ),
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigate_action_server = navigate_server.navigate_action_server:main",
            "fake_odom = navigate_server.odom_sim:main",
        ],
    },
)
