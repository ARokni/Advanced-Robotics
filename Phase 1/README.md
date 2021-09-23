# Phase I

* In this project I have implemented a python code in oredr to identify the **`Motion-Model`**  and **`Sensor-Model`** of a robot named [**`Vector`**](https://www.amazon.com/Vector-Robot-Anki-Hangs-Helps/dp/B07G3ZNK4Y). 


    # Motion Model

    - For the identification of the probailistic **Motion Model** of the robot, I applied an **`Odemotery Model`**.

    - This model consider two major soruce of erros: **errors due to the translational movements** and **errors due to the rotational movements**. 
    
    - **`Sampling Algorithms`** has been applied in order to determine the motion model of the robot.

    - Utilizing a simple control (**`proportional Control`**), I recorded the data for different control inpusts: **5 [CM]**, **10[CM]**, and **20[CM]**  translational displacements; and **15[deg]**, **20[deg]**, and **30[deg]** rotational displacements. The measured data has been employed for paramter identification of the **`Odometry Model`** of the robot.
    
    - Since the experiment has been concducted in **`Gzebo`**, the parameters related to the uncertainty part has been obtained nearly zero.

    - An intersted reader can refer to [[1]](#1), chaper **Robot Motion** for further readings.

    
    # Sensor Model
    - This robot is equiped with an **Infrared Laser Scanner** that lets it to track distance and find obstacles, which are located at most **40[CM]** distance from the robot.

    - For the identification of the  probabilistc **Sensor Model** of the robot, I applied a **`Range Finder Model`**. In this model four types of measurement erros are considered: **measurement noise**, **errors due to unexpected objects**, **errors due to failures to detect objects**, and **random unexplained noise**.

    - The **Sensor Model** experiment has been conducted by observing the measurement of the sensor of the robot at various distances from a particular obstacle. The records are saved for **10[CM]**, **20[CM]**, and **30[CM]** distances.

    -An intersted reader can refer to [[1]](#1), chaper **Measurements** for further readings.

    # Codes
    - The codes for **Measurement Model** experiment are provided in [odometry.py](https://github.com/ARokni/Advanced-Robotics/blob/main/Phase%201/Motion_Model/odometry.py) and [velocity_angular.py](https://github.com/ARokni/Advanced-Robotics/blob/main/Phase%201/Motion_Model/velocity_angular.py).

    - The codes for **Sensor Model** experiment are provided in [laser.py](https://github.com/ARokni/Advanced-Robotics/blob/main/Phase%201/Sensor_Model/laser.py).

    - A linear regression code is provided in[Learning_Parameters.py](https://github.com/ARokni/Advanced-Robotics/blob/main/Phase%201/Sensor_Model/learning_parameters.py)  for determining the sensor model parameters.




## References
<a id="1">[1]</a> 
[Bongard, J., 2008. Probabilistic robotics. sebastian thrun, wolfram burgard, and dieter fox.(2005, mit press.) 647 pages.](https://www.amazon.com/Probabilistic-Robotics-INTELLIGENT-ROBOTICS-AUTONOMOUS/dp/0262201623)






