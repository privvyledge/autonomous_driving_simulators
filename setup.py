from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_driving_simulators'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
]


def package_files(data_files, directory_list):
    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=package_files(data_files, ['data/']),
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Boluwatife Olabiran',
        maintainer_email='bso19a@fsu.edu',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'coupled_kinematic_casadi = autonomous_driving_simulators.coupled_kinematic_casadi:main',
                'coupled_kinematic_do_mpc = autonomous_driving_simulators.coupled_kinematic_do_mpc:main',
                'coupled_kinematic_acados = autonomous_driving_simulators.coupled_kinematic_acados:main',
                'kinematic_dompc_simulator = autonomous_driving_simulators.simulator.do_mpc.do_mpc_simulator_node:main',
                'kinematic_acados_simulator = autonomous_driving_simulators.simulator.acados.acados_simulator_node:main',
                'twist_to_ackermann = autonomous_driving_simulators.twist_to_ackermann_drive:main',
                'waypoint_recorder = autonomous_driving_simulators.waypoint_recorder:main',
                'waypoint_loader = autonomous_driving_simulators.waypoint_loader:main',
            ],
        },
)
