from setuptools import setup
import glob
import os

package_name = 'configuration'

def genDataFilesDirectoryRecursive(directory):
    ''' Adapted from https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/?answer=397410#post-id-397410 '''
    paths_dict = {}
    data_files = []
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

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob.glob("launch/*.py")),
    # ('share/' + package_name + '/config', glob.glob("config/*")),
]
data_files += genDataFilesDirectoryRecursive("config")
data_files += genDataFilesDirectoryRecursive("models")

# print("GOT DATA FILES:\n" + str(data_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony.goeckner@northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
)
