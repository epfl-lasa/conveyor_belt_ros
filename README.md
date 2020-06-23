# conveyor_belt_ros

## NOTE

**The terminal of the conveyor belt must be in *Extern Command 2 (RS232)* Mode. This is done by pressing the bottom window in the terminal of the conveyor belt**

#### Dependencies

Before using this package, make sure you installed ros-indigo-serial:

```
$ sudo apt-get install ros-indigo-serial
```

## Serial connection

If you have trouble connecting to the conveyor belt ("Unable to open port"), you can follow these instructions

```
$ ls -l /dev/ttyS0
```
Normally, the conveyor belt would be connected to 'S0', and you need to modify the premissions:
```
$ sudo chmod o+rw /dev/ttyS0
```

