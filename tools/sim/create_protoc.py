from distutils.command.build_py import build_py
import os
from setuptools import setup
import argparse

parser = argparse.ArgumentParser(
        description='ProtoBuf file compiler')

parser.add_argument(
    '--file',
    help='File to compile')

args = parser.parse_args()

class GenerateBindings(build_py):
    os.system('echo generating bindings')
    os.system('mkdir -p generated')
    os.system('protoc -I=proto-schema --python_out=generated proto-schema/' + args.file)
    os.system('echo generated bindings')

setup(name='ProtoBuf file compiler',
      version='0.0.1',
      cmdclass={'build_py': GenerateBindings}
     )