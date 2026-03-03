"""Setup script for endoscope_control package."""

from setuptools import setup, find_packages

setup(
    name="endoscope_control",
    version="0.1.0",
    description="Language-to-motion pipeline for G1 humanoid endoscope assistant",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.24",
        "pyyaml>=6.0",
        "pyzmq>=25.0",
        "pydantic>=2.0",
    ],
)
