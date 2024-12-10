from setuptools import find_packages, setup

package_name = 'pi_user_input_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gkouretas',
    maintainer_email='thegreekgeorge07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_user_input = pi_user_input_node.test_user_inputs:main',
            'user_input_node = pi_user_input_node.user_input_node:main'
        ],
    },
)
