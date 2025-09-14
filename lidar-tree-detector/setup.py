from setuptools import setup, find_packages

setup(
    name='lidar-tree-detector',
    version='0.1.0',
    author='Your Name',
    author_email='your.email@example.com',
    description='A package for detecting tree sizes using LiDAR data from drones.',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'laspy',
        'scikit-learn',
        'pandas'
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.6',
)