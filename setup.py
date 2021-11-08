import numpy
import setuptools
from Cython.Build import cythonize
from setuptools import Extension, setup

setup(
	name="zmotion_py",
	version="0.0.1",
	description="Python Bindings for the library of the ZMotion control card.",
	url='https://github.com/embodiedintelligence/zmotion-py',
	packages=setuptools.find_packages(),
	python_requires='>=3, <4',
	ext_modules=cythonize(
		[
			Extension(
				# the extension name
				"zmotion_py",
				# the Cython source and additional C++ source files
				sources=["zmotion_py.pyx"],
				# Cython module is linked against
				libraries=["zmotion"],
				include_dirs=["lib", numpy.get_include()],
				# libname.so, looked in "."
				library_dirs=["lib"],
				# generate and compile C++ code
				language="c++",
			)
		]
	),
)
