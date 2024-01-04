from setuptools import setup, Extension
import pybind11
import os
import subprocess

def find_opencv():
    # Attempt to find OpenCV using pkg-config
    try:
        opencv_cflags = subprocess.check_output("pkg-config --cflags opencv4", shell=True).decode().strip()
        opencv_libs = subprocess.check_output("pkg-config --libs opencv4", shell=True).decode().strip()

        # Extracting include and library paths
        include_dirs = [path[2:] for path in opencv_cflags.split() if path.startswith('-I')]
        library_dirs = [path[2:] for path in opencv_libs.split() if path.startswith('-L')]
        libraries = [lib[2:] for lib in opencv_libs.split() if lib.startswith('-l')]

        return include_dirs, library_dirs, libraries
    except subprocess.CalledProcessError:
        # Fallback to common paths
        common_paths = ['/usr/local', '/opt/local', '/usr']
        for path in common_paths:
            include_path = os.path.join(path, 'include')
            lib_path = os.path.join(path, 'lib')
            if os.path.exists(include_path) and os.path.exists(lib_path):
                return [include_path], [lib_path], ['opencv_core', 'opencv_imgproc', 'opencv_highgui']
    return [], [], []

opencv_include_dirs, opencv_library_dirs, opencv_libraries = find_opencv()

ext_modules = [
    Extension(
        'person_tracker',
        ['pybind.cpp'],
        include_dirs=[pybind11.get_include(), '/home/pi/pyboostcvconverter/include', '/home/pi/drone/person-tracking-drone/tensorflowMethod/.venv/lib/python3.9/site-packages/numpy/core/include', '/home/pi/drone/person-tracking-drone/tensorflowMethod/person_detection'] + opencv_include_dirs,
        library_dirs= opencv_library_dirs + ['/usr/local/lib'],
        libraries= opencv_libraries + ["tensorflow-lite"],
        language='c++'
    ),
]

setup(
    name='person_tracker',
    version='1.1',
    ext_modules=ext_modules,
)
