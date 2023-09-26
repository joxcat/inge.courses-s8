from setuptools import setup

package_name = 'jpjc_qr'

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
    maintainer='triton_08',
    maintainer_email='triton_08@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr = jpjc_qr.qr:main',
            'qr_vision = jpjc_qr.qr_vision:main',
            'qr_follower = jpjc_qr.qr_follower:main',
        ],
    },
)
