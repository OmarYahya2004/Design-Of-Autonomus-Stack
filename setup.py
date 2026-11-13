import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'csen934_project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'Webots_Worlds_Controller/Worlds'), glob('Webots_Worlds_Controller/Worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssif',
    maintainer_email='youssif@todo.todo',
    description='CSEN934 Project Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #'ui_teleop = csen934_project_pkg.ui_teleop_node:main',
            'control = csen934_project_pkg.control:main',
            'localization_add_noise = csen934_project_pkg.localization_add_noise:main',
            'localization_ekf_node = csen934_project_pkg.localization_ekf_node:main',
            'yaw_plot_publisher = csen934_project_pkg.yaw_plot_publisher:main',
            'localization_plots = csen934_project_pkg.localization_plots:main'
        ],
    },
)
