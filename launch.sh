#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
cd /data/notebooks
jupyter notebook --port=8888 --no-browser --ip=0.0.0.0 --allow-root --NotebookApp.password='password' --NotebookApp.token='password'
