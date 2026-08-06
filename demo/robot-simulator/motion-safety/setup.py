from setuptools import setup

setup(
    name="rovera_motion_safety",
    version="1.0.0",
    py_modules=["safety_core", "safety_node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rovera_motion_safety"]),
        ("share/rovera_motion_safety", ["package.xml"]),
    ],
    entry_points={"console_scripts": ["safety_node=safety_node:main"]},
)
