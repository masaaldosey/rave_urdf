# raveurdf

an [openRAVE](https://openrave.org/) plugin to load a given *URDF* file.

## What does it do?

- loads a given *URDF* file into openRAVE as `OpenRAVE::RobotBase`.
- exploits builtin openRAVE helpers for *Collada* export.

## Demo

the plugin uses a Franka Robot for demonstration purposes.


## Dependencies

the plugin needs the following to be installed on your machine.

- [openRAVE](https://github.com/Ahmed-M-Naguib/openrave/tree/agile-changes).
  Please install the dependencies listed in the [`ci.yaml`](https://github.com/Ahmed-M-Naguib/openrave/blob/agile-changes/.github/workflows/ci.yaml) before building from source.
- TinyXML2  
    ```bash
    $ git clone https://github.com/leethomason/tinyxml2.git
    $ cd tinyxml2
    $ mkdir build && cd build
    $ cmake ..
    $ make
    $ sudo make install
    ```
- `urdf` libraries
    ```bash
    $ sudo apt install liburdf*
    ```
