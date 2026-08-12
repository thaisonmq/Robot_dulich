from setuptools import setup

setup(
    name="rovera_navigation_adapter",
    version="1.0.0",
    py_modules=["adapter_node", "navigation_core", "sensor_normalizer", "speed_profiles"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rovera_navigation_adapter"]),
        ("share/rovera_navigation_adapter", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "adapter_node=adapter_node:main",
            "sensor_normalizer=sensor_normalizer:main",
        ]
    },
)
