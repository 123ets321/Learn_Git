from setuptools import find_packages, setup

package_name = 'vtol_stack'

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
    maintainer='hakkam',
    maintainer_email='sashank.erukala@theskytex.com',
    description='TODO: Package description',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dive_vtol = vtol_stack.dive_new:main",
            "vtol_comms = vtol_stack.comms_new:main"
        ],
    },
)
