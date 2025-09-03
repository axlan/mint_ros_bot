from setuptools import find_packages, setup

package_name = 'relative_tag_test'

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
    maintainer='jdiamond',
    maintainer_email='jonathan@pointonenav.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
                'console_scripts': [
                    'relative_tag_positions = relative_tag_test.relative_tag_positions:main',
                ],
    },
)
