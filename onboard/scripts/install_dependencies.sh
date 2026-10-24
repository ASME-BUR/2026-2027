#!/bin/bash
# Carried over from bur-onboard unchanged except for vcs2l -> vcs.
info () { echo -e "\e[32m[INFO]\e[00m " $@; }

info "Checking connection to http://github.com..."
wget -q --timeout 5 --spider http://github.com

if [ $? -eq 0 ]; then
    info "Connected, installing dependencies..."
    mkdir -p src/submodules
    vcs import --input required.repos src/submodules
else
    info "No connection, skipping..."
fi

rosdep install --from-paths src --ignore-src -r -y
