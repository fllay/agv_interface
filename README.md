# agv_interface


```pico /home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml```
```
bridge:
- factory: mqtt_bridge.bridge:RosToMqttBridge
  msg_type: visualization_msgs.msg:Marker
  topic_from: /eru/visualization_marker
  topic_to: /eru/visualization_marker
deserializer: msgpack:loads
mqtt:
  client:
    protocol: 4
  connection:
    host: 192.168.68.107
    keepalive: 60
    port: 1883
  private_path: device/001
serializer: msgpack:dumps
```

```sudo pip install pyyaml```

```roslaunch mqtt_bridge demo.launch```


```pico /etc/ros/melodic/linorobot.d/bringup.launch ```

``` 
  <arg name="use_tls" default="false" />
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam command="delete" param="" />
    <rosparam command="load" file="$(find mqtt_bridge)/config/demo_params.yaml" />
    <rosparam if="$(arg use_tls)" command="load" ns="mqtt" file="$(find mqtt_bridge)/config/tls_params.yaml" />
  </node>

```


#Server side MQTT


```pico /home/pi/linorobot_ws/src/mqtt_bridge/config/demo_params.yaml```
``` 
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
  private_path: device/001
#serializer: json:dumps
#deserializer: json:loads
bridge:
  # ping pong

  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: visualization_msgs.msg:Marker
    topic_from: /eru/visualization_marker
    topic_to: /eru/visualization_marker


  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: visualization_msgs.msg:Marker
    topic_from: /bot1/visualization_marker
    topic_to: /bot1/visualization_marker


```




