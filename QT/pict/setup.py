import setuptools

setuptools.setup(
    name='pict',
    version='0.0.1',
    packages=setuptools.find_packages(),
    install_requires=[
        'PyQt6',
        'imagesize'
    ],
    entry_points={
        'gui_scripts': [
            'pict = main:main',
        ],
    },
    python_requires='>=3.6',
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    package_data={
    '': ['resources/*'],
    },
    include_package_data=True
)
