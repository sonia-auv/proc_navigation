# Proc Navigation

This node is part of S.O.N.I.A's AUV7 software. Is use to computed the AUV global pose in the North East Down (N.E.D) referential.

## Getting Started

```bash
$ git clone https://github.com/sonia-auv/proc_navigation
```

### Prerequisites

You must install S.O.N.I.A's ROS repositories to use this module.

S.O.N.I.A's installation instruction are available at [SONIA's Installation](https://sonia-auv.readthedocs.io/user/installation/)

### Development

##### Devices

The devices used to find out the globe pose of the AUV is an [IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit) and a [DVL](https://en.wikipedia.org/wiki/Acoustic_Doppler_current_profiler#Bottom_tracking).

In AUV7 software the IMU and DVL information is provide by [provider_imu](https://github.com/sonia-auv/provider_imu) node and [provider_dvl](https://github.com/sonia-auv/provider_dvl) node. 

##### Input 

Input information is provide to the node using this ROS msgs :

* IMU information [sensor_msgs IMU](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html) 
* DVL information [geometry_msgs TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html) and [sensor_msgs FluidPressure](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/FluidPressure.html)

##### Output

Output information is provide to other node using this ROS msg :

* Pose and Twist information [nav_msgs Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)

Orientation information is not output as quaternion but as euler angle.

##### Algorithms

The DVL information (velocity) is transform to position (xyz) using [numerical integration](https://en.wikipedia.org/wiki/Numerical_integration). We use the [Runge-Kutta](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods) methods to reduce error during the integration process. A standard numerical integration method is also available.

The IMU information ([quaternion](https://en.wikipedia.org/wiki/Quaternion)) is transform to [euler angles](https://en.wikipedia.org/wiki/Euler_angles).

The Pose, based on N.E.D referential, is find out using [rotation matrix](https://en.wikipedia.org/wiki/Rotation_matrix) transformation.

An [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) is implemented in this node. Is used as a state estimator on AUV pose. Implementation is based on this repository [TinyEKF](https://github.com/simondlevy/TinyEKF). 

## Running the tests

No tests implemented yet. You can run a [simulation](https://github.com/sonia-auv/proc_control/blob/develop/script/AuvSimulation.py) to test new navigation algorithms. It's highly recommended to test the node on a real system.

## Built With

* [ROS](http://www.ros.org/) - ROS Robotic Operating System
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - Linear algebra

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Thibaut Mattio** - *Initial work* - [tmattio](https://github.com/tmattio)
* **Etienne Boudreault-Pilon** - *Initial work* - [etiennepilon](https://github.com/etiennepilon)
* **Jérémie St-Jules Prévôt** - *Initial work* - [jsprevost](https://github.com/jsprevost)

### Contributors
* **Francis Massé** - [fmassey](https://github.com/fmassey)
* **Olivier Lavoie** - [olavoie](https://github.com/olavoie)

## License

This project is licensed under the GNU GPL V3 License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Extended Kalman filter implementation is based on [TinyEKF](https://github.com/simondlevy/TinyEKF). 