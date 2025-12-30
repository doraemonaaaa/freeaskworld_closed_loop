from setuptools import setup

package_name = 'vln_connector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='PENG Yuhang',
    maintainer_email='3478428491@qq.com',
    description='VLN RGBD Connector Node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rgbd_connector = vln_connector.rgbd_connector:main',
        ],
    },
)
