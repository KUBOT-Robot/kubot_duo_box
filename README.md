# kubot_duo_box
這是KUBOT Robot第二下位機控制package.

目前可以：

- [x] 控制ws2812燈條X4 (同步)
- [x] 控制伺服馬達X2 (異步)

### 快速引導目錄
 - [安裝](#install)
 - [控制燈條說明](#led-topic)
 - [控制伺服舵機說明](#servo-motor-topic)

# Install

1. 進行控制板初始化, 將設定裝置rules. 
   ```sh
   # cd This package.
   sudo ./kubot_duo_box_init_env.sh
   ```

2. 進行catkin_make
   ```sh
   # cd YOUR ROS1 workspace
   catkin_make
   ```

3. 啟動第二下位機
   ```sh
   roslaunch kubot_duo_box bringup.launch
   ```

4. rosotpic： 
   ```sh
   rostopic pub /led_control 
   ```

# LED Topic

ROS Pub `/led_control` 用於控制下位機燈條狀態, 目前版本4條同步.

ROS Sub `/led_status` 用於確認下位機燈條狀態是否更新.

Example：
   ```sh
   rostopic pub /led_control kubot_duo_msgs/RawLeds "header:
   seq: 0
   stamp: {secs: 0, nsecs: 0}
   frame_id: ''
   ledNum: 2
   led_brightness: 200
   led_speed: 200
   led_color_r: 200
   led_color_g: 0
   led_color_b: 200
   led_mode: 10" 
   ```

|     參數名     |              含意               | 有效範圍 |
| :------------: | :-----------------------------: | :------: |
|     ledNum     | 第N個燈條(目前無意義, 全部更新) |   0~4    |
| led_brightness |             led亮度             |  0~255   |
|   led_speed    | 動態變化速度(無關的狀態則無意)  |  0~255   |
|  led_color_r   |         變化顏色RGB之R          |  0~255   |
|  led_color_g   |         變化顏色RGB之G          |  0~255   |
|  led_color_b   |         變化顏色RGB之B          |  0~255   |
|    led_mode    |     閃爍模式(看下方Effects)     |   0~64   |

### 每次Topic更新最好間隔5秒以上！
### 部份狀態更新時間超過5s, 終端會跳`timeout`, 期間不要發布新的狀態, 避免異常.


Effects
-------

1. **Static** - No blinking. Just plain old static light.
2. **Blink** - Normal blinking. 50% on/off time.
3. **Breath** - Does the "standby-breathing" of well known i-Devices. Fixed Speed.
4. **Color Wipe** - Lights all LEDs after each other up. Then turns them in that order off. Repeat.
5. **Color Wipe Inverse** - Same as Color Wipe, except swaps on/off colors.
6. **Color Wipe Reverse** - Lights all LEDs after each other up. Then turns them in reverse order off. Repeat.
7. **Color Wipe Reverse Inverse** - Same as Color Wipe Reverse, except swaps on/off colors.
8. **Color Wipe Random** - Turns all LEDs after each other to a random color. Then starts over with another color.
9. **Random Color** - Lights all LEDs in one random color up. Then switches them to the next random color.
10. **Single Dynamic** - Lights every LED in a random color. Changes one random LED after the other to a random color.
11. **Multi Dynamic** - Lights every LED in a random color. Changes all LED at the same time to new random colors.
12. **Rainbow** - Cycles all LEDs at once through a rainbow.
13. **Rainbow Cycle** - Cycles a rainbow over the entire string of LEDs.
14. **Scan** - Runs a single pixel back and forth.
15. **Dual Scan** - Runs two pixel back and forth in opposite directions.
16. **Fade** - Fades the LEDs on and (almost) off again.
17. **Theater Chase** - Theatre-style crawling lights. Inspired by the Adafruit examples.
18. **Theater Chase Rainbow** - Theatre-style crawling lights with rainbow effect. Inspired by the Adafruit examples.
19. **Running Lights** - Running lights effect with smooth sine transition.
20. **Twinkle** - Blink several LEDs on, reset, repeat.
21. **Twinkle Random** - Blink several LEDs in random colors on, reset, repeat.
22. **Twinkle Fade** - Blink several LEDs on, fading out.
23. **Twinkle Fade Random** - Blink several LEDs in random colors on, fading out.
24. **Sparkle** - Blinks one LED at a time.
25. **Flash Sparkle** - Lights all LEDs in the selected color. Flashes single white pixels randomly.
26. **Hyper Sparkle** - Like flash sparkle. With more flash.
27. **Strobe** - Classic Strobe effect.
28. **Strobe Rainbow** - Classic Strobe effect. Cycling through the rainbow.
29. **Multi Strobe** - Strobe effect with different strobe count and pause, controlled by speed setting.
30. **Blink Rainbow** - Classic Blink effect. Cycling through the rainbow.
31. **Chase White** - Color running on white.
32. **Chase Color** - White running on color.
33. **Chase Random** - White running followed by random color.
34. **Chase Rainbow** - White running on rainbow.
35. **Chase Flash** - White flashes running on color.
36. **Chase Flash Random** - White flashes running, followed by random color.
37. **Chase Rainbow White** - Rainbow running on white.
38. **Chase Blackout** - Black running on color.
39. **Chase Blackout Rainbow** - Black running on rainbow.
40. **Color Sweep Random** - Random color introduced alternating from start and end of strip.
41. **Running Color** - Alternating color/white pixels running.
42. **Running Red Blue** - Alternating red/blue pixels running.
43. **Running Random** - Random colored pixels running.
44. **Larson Scanner** - K.I.T.T.
45. **Comet** - Firing comets from one end.
46. **Fireworks** - Firework sparks.
47. **Fireworks Random** - Random colored firework sparks.
48. **Merry Christmas** - Alternating green/red pixels running.
49. **Fire Flicker** - Fire flickering effect. Like in harsh wind.
50. **Fire Flicker (soft)** - Fire flickering effect. Runs slower/softer.
51. **Fire Flicker (intense)** - Fire flickering effect. More range of color.
52. **Circus Combustus** - Alternating white/red/black pixels running.
53. **Halloween** - Alternating orange/purple pixels running.
54. **Bicolor Chase** - Two LEDs running on a background color.
55. **Tricolor Chase** - Alternating three color pixels running.
56. **TwinkleFOX** - Lights fading in and out randomly.
57. thru 63. **Custom** - Up to eight user created custom effects.


# Servo Motor Topic

ROS Pub `/servo_control` 用於控制伺服舵機狀態, 目前版本2顆獨立控制.
ROS Sub `/servo_status` 用於確認伺服狀態.

Example：
   ```sh
   rostopic pub /servo_control kubot_duo_msgs/RawServop "header:
   seq: 0
   stamp: {secs: 0, nsecs: 0}
   frame_id: ''
   servoNum: 1
   servo_angle: 90" 
   ```

|     參數名     |              含意               | 有效範圍 |
| :------------: | :-----------------------------: | :------: |
|     servoNum     | 第N顆伺服舵機 |   0~2    |
| servo_angle |   舵機角度     |  0~180   |
