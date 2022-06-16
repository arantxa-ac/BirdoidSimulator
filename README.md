WELCOME TO THE BOID SIMULATOR

To open the panel press P.
To change alignment constant modify ka in the panel.
To change cohession constant modify kc in the panel.
To change separation constant modify ks in the panel.

Documentation
-------------

https://lakin.ca/givr/

Compilation
-----------

## How to Install Dependencies (Ubuntu)

    sudo apt install cmake build-essential

## How to Build

    cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release
    cmake --build build

## How to Run

    build/simple
