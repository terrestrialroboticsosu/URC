# Driver Station Code Setup

The driver station for lunabotics is written in python so that first thing you will need to do is [install python](https://www.python.org/) for your operating system. (Will only work with Python versions from 3.6 - 3.11)

All of the dependencies are configured in a virtual environment so in order to run the code you need to do the following after opening a terminal inside the DriverStation folder in this repository.

To activate the virtual environment and run the program do the following for linux/macOS

```
python3 -m venv venv
source venv/bin/activate
```

and for windows do
```
python -m venv venv
```

or
```
py -m venv venv
```

then this
```
.\venv\Scripts\activate
```

install dependencies

```
pip install -r requirements.txt
```

then to run the driver station

```
python3 ds.py
```

and when you are done working close the virtual environment with

```
deactivate
```
