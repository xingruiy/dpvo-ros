from setuptools import find_packages, setup
from torch.utils.cpp_extension import CUDAExtension, BuildExtension
import os.path as osp

ROOT = osp.abspath(osp.dirname(__file__))
package_name = 'cuda_ba'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    ext_modules=[
        CUDAExtension(
            'cuda_ba_ext',
            sources=[
                'cuda_ba/ba.cpp',
                'cuda_ba/ba_cuda.cu',
                'cuda_ba/block_e.cu'],
            extra_compile_args={
                'cxx':  ['-O3'],
                'nvcc': ['-O3'],
            },
            include_dirs=['/usr/include/eigen3']
        ),
    ],
    cmdclass={
        'build_ext': BuildExtension
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='64749983+xingruiy@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
