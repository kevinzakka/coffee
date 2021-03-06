#!/bin/bash

python --version

# Bash settings: fail on any error and display all commands being run.
set -e
set -x

black .
isort . --profile black

mypy .
flake8 .
