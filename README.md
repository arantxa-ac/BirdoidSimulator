Birdoid Simulator
-----------------
Assignment for Fundamentals of Computer Animation course. The purpose of this assignment was to simulate a flock of birds using collision avoidance and attraction forces. Reynolds’s model of schools, flocks, and herds was used to accomplish the task.

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
