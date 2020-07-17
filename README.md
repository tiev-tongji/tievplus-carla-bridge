# tiev-sim-cppClient

a cpp bridge between Carla and tiev-plus

## dependencis
1. [GeographicLib]() for caluculating the UTM and GPS coordinates
2. [rapidjson (already in src)]() for setting parameters
3. clang++-8.01 (the same as you compiled carla and LibCaral)
4. [cmake after version 3.0]()
5. [zeocm]()


## tips
1. ubuntu default udp buffer is too small to run tiev-sim effciently, to enlarge kernel buffer size:

```
# sudo vi /etc/sysctl.conf and add below sentences to the bottom
net.core.rmem_max=104857600
net.core.rmem_default=104857600
net.core.wmem_max=104857600
net.core.wmem_default=104857600
```
2. when compiled carla from source, do NOT use `sudo` or there may be some errors which need to change files' owner to fix.
3. when compiled carla, if `make launch` throw an error that shows `UE4_ROOT` cannot be found but it has indeed already be set in `~/.bashrc` correctly, add `UE4_ROOT=PATH.../UnrealEngine-4.24` to the first line of `Carla/Util/BuildTools/BuildCarlaUE4.sh`

## build
1. clone this repo to the carla installation path, such as `~/carla/`
2. make sure `LibCarla` been built and can be found in `....../carla/PythonAPI/carla/dependencies`, there shall be `include` and `lib` folder
3. cmake and make, compiler should be assigned to clang++-8
4. several macros to control this bridge

to change between LCM and ZCM, in `MessageManager/MessageManger.h`
```
#define USE_LCM
#define USE_ZCM
```
to change ego car's controller between carla default agent and tiev-plus, in `tiev-sim.cpp`
```
#define HIL_MODE
#define AUTOPILOT_MODE
```
to change message publishing mode between async and sync, or to test how much time spent for packing and publishing of every message (sync mode)
``` 
#define ASYNC_MODE
#define SYNC_MODE
#define OPT_TIME_TEST
```

## develop
to add a new feature supporting a new message, follow belowing guide: 

Here we assume that the new zcm/lcm message named `new_msg` 

1. in `MessageManger.h` and `MessageManger.cpp`, add `pub_new_msg_loop(int freq)` and `publish_new_msg()`, in `publish_all_async()`, add a new line to rigitster the thread function.
2. using CarlaLib API to implement the `pack_new_msg()` function for packing new message.
3. if you have used a carla virtual sensor to implement `pack_new_msg()`, in `tiev-sim.cpp` a callback function for listening sensor data is needed.
4. if you haved used `carla::client::World`'s method such as `GetActors()` to implement your packing function, you need to call this `GetActors()` in `tick()` function of `MyWorld`.

## Todo
1. adjust coordinates from rear-axle to front axle.
2. point cloud based fusionmap
3. additional messages: roadmarking, traffic sign, traffic light, rain detection.
4. how to create a display window like pygame in c++? or use a python script to get ego car's ID and attach a spectator?