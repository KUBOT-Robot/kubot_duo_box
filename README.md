# kubot_duo_box
這是用於測試第二下位機之通訊

- [x] 控制ws2812燈條
- [x] 控制伺服馬達

### STM32F103C8T6

```sh
sudo chmod 777 /dev/ttyUSB0
```

```sh
roslaunch kubot_duo_box bringup.launch
```

```sh
rostopic pub /led_control 



```
