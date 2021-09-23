
# Phase II

* In this project I have implemented **`Particle Filter`**([[1]](#1), p.200) algorithm to solve the **`Mobile Robot Global Localization`** and **`Robot Kidnapping`** problems. 

* Given a **`map`** of the environment, this algorithm runs repeatedly to converge to the final location of the robot. The algorithm considers the measurements and motion of the robot in order to update the particles at each iteration.

    # Localization

    * **Motion Model**: As mentioned in the [Phase I](https://github.com/ARokni/Advanced-Robotics/tree/main/Phase%201) of this project, I have applied a sampling algorith based on **`odometry`** model for the motion of the robot.

    * **Sensor Model**: On the ground of the  [Phase I](https://github.com/ARokni/Advanced-Robotics/tree/main/Phase%201) first I utilzied the **`Beam Range Finder`**([[1]](#1), p.124) algorithm for the **`Sensor (measurement)`** part of the robot. However,  the algorithm is time consuming since it employs **ray-csating** ([[1]](#1), p.124) in order to find each particle wieght. As a result, I utilized another algorithm, **`Likelihood Field Range Finder Model`** ([[1]](#1), p.139), which is faster than the **Beam Range Finder** method.


    * **Resampling**: I employed **`Roulette Wheel`** method in the resampling part of the **Particle Filter**.

    * In order to robuts the **Localization** and  cope with the **Kidanpping Problem** , I regenerate five percent of the whole particles randomly after the resampling part.

    # Kidnapping

    * I have utilized two different algorithm for the kinapping part. 

    * First, I employed a particular algorithm, named **Augmented**, which is described in detail in [[1, p.206]](#1). 

    * Second, derive an algorithm on my own: whenever the weights of particles become less than the half of the previous iteration of the algorithm, the code detect the situation as **Kidnapping** and by regenrating the 90 percent of the particles aims to cope with the **Kidnapping**.





## References
<a id="1">[1]</a> 
[Bongard, J., 2008. Probabilistic robotics. sebastian thrun, wolfram burgard, and dieter fox.(2005, mit press.) 647 pages.](https://www.amazon.com/Probabilistic-Robotics-INTELLIGENT-ROBOTICS-AUTONOMOUS/dp/0262201623)



