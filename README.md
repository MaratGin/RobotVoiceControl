# Control Wheeled mobile robot using voice commands


## Как всё запускать:
- Запускаем основной сервер
    1. `cd voice_control`
    2. `cd voice_control`
    3. `python3 __init__.py`
- Запускаем ROS прослойку
    1. cd `robot_voice_ws`
    2. `colcon build`
    3. `source install/setup.bash`
    4. `cd src/scripts`
    5. `python3 bridge_service.py` (Для обычных сообщений Twist)
    5. Если не работает предыдущий вариант`python3 bridge_service_stamped.py` (для TwistTtamped)
- Запускаем симуляцию
    1. `turtlebot4_gz_bringup turtlebot4_gz.launch.py`
    2. (Можно запускать любого робота с diff_drive_controller (необязательно turtlebot4) и всё равно будет работать)

## Не забудьте поменять айпишники, если они отличаются (в файле voice_control/voice_control/configs.py ) и топики (в файлах bridge_service.py/bridge_service_stamped.py)