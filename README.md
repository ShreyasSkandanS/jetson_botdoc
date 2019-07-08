# Jetson BotDoc

### Monitoring ROS Node for Jetson Xavier

![botdoc](/imgs/botdoc.jpg)

Usage:
```
roslaunch jetson_botdoc monitor.launch
```

Output:
```
rostopic echo /jetson_botdoc/health_report
```

You can view this data using *rqt_graph* or *rqt_bag*

Example:
![botdoc](/imgs/botdocgraph.png)


### References:

Lots of tricks and methods borrowed from:
1. https://github.com/jetsonhacks/gpuGraphTX
2. https://devtalk.nvidia.com/
