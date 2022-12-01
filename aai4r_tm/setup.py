from setuptools import setup

package_name = 'aai4r_tm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='minsu@etri.re.kr',
    description='AAI4R Task Manager',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tm_node = aai4r_tm.tm_node:main',
            'deliverybot_node = aai4r_tm.deliverybot_node:main',
            'tablebot_node = aai4r_tm.tablebot_node:main'
        ],
    },
)
