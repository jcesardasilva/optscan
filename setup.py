#!/usr/bin/env python
import setuptools

meta = {}
exec(open('./optscan/version.py').read(), meta)
meta['long_description'] = open('./README.md').read()

setuptools.setup(
    name='optscan',
    version=meta['__version__'],
    description='Tool to optimize scan trajectory',
    long_description=meta['long_description'],

    author='Julio C. da Silva, Cyril Guilloud',
    author_email='jdasilva@esrf.fr',
    url='https://github.com/jcesardasilva/optscan',
    python_requires='>=3.7',
    install_requires=[
        'matplotlib >= 3.1.1',
        'numpy >= 1.16.4',
        'ortools >= 7.2.6977'
    ],
    packages=setuptools.find_packages(),
    license='GNU General Public License, Version 3.0',
)
