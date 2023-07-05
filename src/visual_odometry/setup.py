from setuptools import setup

package_name = 'visual_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dawoood',
    maintainer_email='dawoood@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "v_odometry = visual_odometry.visual_od: main",
            "recorder = visual_odometry.record: main"
        ],
    },
)
