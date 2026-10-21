from setuptools import find_packages, setup

package_name = 'practica_1'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ignacio',
    maintainer_email='ignacio@todo.todo',
    description='Extracción y Análisis de Datos Cinemáticos del Robot UR3e con ROS 2. Práctica 1',
    license='TODO',
    entry_points={
        'console_scripts': [
        ],
    },
)
