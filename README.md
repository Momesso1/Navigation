Entre as linhas 400 e 534 o D* é feito de fato. O A* é feito normalmente na primeira vez, mas depois é checado se um obstaculo apareceu no caminho, caso não tenha aparecido, então manda o mesmo caminho, caso tenha aparecido, então um novo caminho é feito com o ponto inicial um pouco antes do obstaculo e o ponto final é um pouco depois do obstaculo, depois disso os caminhos de juntam. Depois disso o caminho reutilizado e os caminhos feitos apenas para contornar os obstaculos são unidos.


Bug 1: 


. install/setup.bash

ros2 launch algorithms bug1.launch.py

Bug 2:

. install/setup.bash

ros2 launch algorithms bug2.launch.py

D*

. install/setup.bash

ros2 launch algorithms d_star.launch.py

Rosbag:

Bug 1:

ros2 bag play src/algorithms/bug1_rosbag/

Bug 2:

ros2 bag play src/algorithms/bug2_rosbag/

Bug 1:

[Screencast from 2025-09-07 09-22-22.webm](https://github.com/user-attachments/assets/84417b6c-d93b-43ba-9606-9bfba654337d)


Bug 2:

[Screencast from 2025-11-08 18-46-49.webm](https://github.com/user-attachments/assets/578a07fc-718f-4fd3-9cac-3f512d866eb9)


D*:

[Screencast from 2025-11-08 18-31-57.webm](https://github.com/user-attachments/assets/9ff9e751-b87d-43d9-a2ad-232b7186d60f)
