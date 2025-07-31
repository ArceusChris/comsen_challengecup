# vtol_condition
* topic : 
```python
rospy.Publisher('/zhihang2025/vtol_land_sub/done', Int8, queue_size=10)
 ```

* condition :

    0xAA : 系统初始化默认状态

    0x01 : 固定翼飞机起飞，飞往第一次巡航起点

    0x02 : 固定翼飞机飞抵第一次巡航起点，开始第一次巡航

    0x03 : 固定翼飞机飞抵第一次巡航终点，下降高度，开始第二次巡航

    0x04 ：固定翼飞机飞抵第二次巡航终点，开始返航着陆

    0x05 ：固定翼飞机成功着落，等待四旋翼飞机接管