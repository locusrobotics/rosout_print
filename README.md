rosout_print
============

This package is a simple utility to read live message from the `/rosout_agg` topic and display those in a log-style formatting.
The command supports colors, level filtering, ROS node filtering and bag reading options. 

Usage:
------
```
$ rosout_print -h
Convert rosout_agg messages to text forms:
  -h [ --help ]                     Produce help message.
  -b [ --bag-file ] arg             Bag file path name (if not specified, live 
                                    ros messages are converted)
  -l [ --level ] arg (=1)           Minimum level to be output (DEBUG=0, 
                                    INFO=1, WARN=2, ERROR=3, FATAL=4)
  -o [ --output-file ] arg          Output file name
  -t [ --topic ] arg (=/rosout_agg) Topic where rosgraph_msgs::Log messages are
                                    published
  -n [ --node-regex ] arg           Regex matching the names of nodes whose 
                                    rosgraph_msgs::Log messages are to be 
                                    processed
  -u [ --utc-time ]                 Output time in UTC
  -e [ --epoch-time ]               Output time in Unix epoch
  -m [ --multiple-lines ]           Line breaks in messages
  -f [ --function-detail ]          Output function name, file, and line number
  --no-color                        do not colorize the output
```

ROS 2 port
---------
A ROS 2 port of this utility is available in the project: [clearpath_ros2cli](https://github.com/clearpathrobotics/clearpath_ros2cli) 

