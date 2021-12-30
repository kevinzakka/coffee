#!/bin/bash

python --version

# Bash settings: fail on any error and display all commands being run.
set -e
set -x

black coffee/
isort coffee/

mypy coffee/
flake8 coffee/
