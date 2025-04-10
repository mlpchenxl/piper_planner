from setuptools import find_packages
from distutils.core import setup

setup(
    name='piper_planning',
    version='1.1.1',
    author='CFY, JZH',
    license="MIT",
    packages=find_packages(),
    author_email='joeey@sjtu.edu.cn',
    description='PIPER_PLANNING: An arm planning for piper',
    install_requires=['mujoco', 'opencv-python'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12'
    ],
)
