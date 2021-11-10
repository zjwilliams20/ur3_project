#!/bin/bash
# /install.bash
# Install necessary model files in the working directory.

CHESSLAB_URL="https://gitioc.upc.edu/jan.rosell/chesslab_setup.git"
MODEL_DIR="/models"
WORLD_DIR="/worlds"

PROJ_DIR="ur3_project"
cdir=$(pwd)
SUB_DIR="submodules"

# Do a partial checkout of a git repository at the specified sub-trees.
# https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository
function git_sparse_clone() (
  rurl="$1" localdir="$2" && shift 2

  mkdir -p "$localdir"
  cd "$localdir"

  git init
  git remote add -f origin "$rurl"

  git config core.sparseCheckout true

  # Loops over remaining args
  for i; do
    echo "$i" >> .git/info/sparse-checkout
  done

  git pull origin master
)


# Verify the current directory is ur3_project
shopt -s extglob           # enable +(...) glob syntax
result=${cdir%%+(/)}    # trim however many trailing slashes exist
result=${result##*/}       # remove everything before the last / that still remains

if [[ $result != $PROJ_DIR ]]; then
    echo "Error: install.bash must be ran from ur3_project directory."
    return
fi

# Reinstall submodules if prompted by user.
if [[ $1 == "-r" ]]; then
    rm -rf $SUB_DIR
fi

if [[ -d $SUB_DIR ]]; then
    echo "chesslab_setup already installed. Use '-r' to reinstall."
    return
fi

git_sparse_clone $CHESSLAB_URL $SUB_DIR $MODEL_DIR $WORLD_DIR

