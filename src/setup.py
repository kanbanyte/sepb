from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="VisionSystem",
    version="0.1.0",
    author="H",
    author_email="h@email.com",
    description="Vision System",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="TO/BE/UPDATED",
    packages=[
        "data_processing",
        "util",
        "models.python"
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: TBA",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.9',
)
