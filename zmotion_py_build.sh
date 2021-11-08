#!/bin/bash
# To use zmotio_py, you should run this shell script first:
# . zmotion_py_build.sh

#fi
CURRENTDIR=${PWD}
CONF_FILE=zmotion_py.conf

echo "zmotion_py SDK configuration and compile script"
echo
echo "Adding the following Arena SDK library paths to /etc/ld.so.conf.d/$CONF_FILE:"
echo "$CURRENTDIR/lib"

# Please change the absolute path to where you libzmotion.so locates.
echo "$CURRENTDIR/lib/" >>/etc/ld.so.conf.d/$CONF_FILE
#echo "INPUT_YOUR_PATH/zmotion-py/lib/" >>/etc/ld.so.conf
ldconfig

echo
echo "Please remember to install these packages if not already installed before proceeding:"
echo "- g++ 5 or higher"
echo "- Cython 0.29.22"
echo "- setuptools 51.3.3"
# build the c++ interface to python interface, using cython.
python ./setup.py install
python setup.py clean --all
#fi
