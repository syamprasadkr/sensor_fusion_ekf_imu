# Sensor Fusion using Extended Kalman Filter for Orientation Estimation

# Important Note: The contents of this repository should not be copied or used without permission. Please see the `Authors` section for contact information.

The objective of this project is to estimate the orientation of a Garmin VIRB camera and IMU unit using Kalman Filter based approaches. In this partcular case, an Extended Kalman Filter has been used with a state space that contains roll, pitch and yaw. The gyroscope has been used to model the process while accelerometer provides the observation. Note that it is not possible to obtain a correct estimate of yaw using this approach. However, yaw has been included in the state space to make use of the short term reliability of gyroscope, in the future parts of the master project. The innovation of yaw does not influence the roll and pitch estmations. The raw sensor data, required for the Filter, is accessed by sending JSON command using cURL package. The response is parsed using Jsoncpp package. 

# Files Required:
1) imu_data.h - Header file for handling data access from sensors.
2) imu_data.cpp - Implementation file for handling data access from sensors.
3) kalman_filter_imu.h - Header file for handling general functions, transfer of information between imu and kalman filter, and central coordination and control.
4) kalman_filter_imu.cpp - Implementation file for handling general functions, transfer of information between imu and kalman filter, and central coordination and control.
5) kf.h - Header file for Kalman filter.
6) kf.cpp - Implementation file for Kalman filter.
7) CmakeLists.txt - The cmake file to configure and generate Makefile required for building and running the code.

# Build Instructions:
1) Make sure all the above files are available in a folder.
2) Open a terminal and type the following commands.
3) $cd /<path_to_folder>
4) $cmake .
5) $make

Executable File: kalman_filter_sf
# Run Instructions:
1) Open a terminal and type the following commands.
2) $cd /<path_to_folder>
3) $./kalman_filter_sf
Output File: log_ornt_<Timestamp>.csv

# Testing and Results:
All the result files are included in the `results` folder.
1) Still test:
For this test, the camera was in a position of zero roll, zero pitch and arbitrary yaw, placed on a level surface. The estimation of these three angles in all the timesteps were logged and plotted. The algorithm estimated all three angles to be close to zero for the entire duration of the test, which is to be expected. The plot for this test is available in the file: phase1_final_still_test.PNG. Unlike the other two tests the results were not compared to the trend indicated by raw gyroscope values and the average value indicated by accelerometer. This has been further explained below.

2) Motion test:
The gyroscope is reliable on short term but it suffers from drift while accelerometer is reliable in long term but suffers from noise. So, the Kalman filter should come up with estimations that don't drift and don't suffer from noise issues. Another way to say it would be that the estimation should pick up the trend of gyroscope while maintaining a (average) value indicated by the accelerometer.

For this test, the camera was picked up from the table after initialization, was moved in random fashion and was placed back on the table. The file: phase1_final_motion_test.xlsx provides the logged values and plots. The Sheets `Pitch` and `Roll` provide the plots of pitch and roll respectively. Rest of the sheets are used to make the calculations and inputs required for the plots. In the plot, `Obs Pitch` line shows pitch values indicated by accelerometer and `Gyro Pitch`  shows pitch values indicated by gyroscope. The `Pitch` indicates the Kalman Filter estimate and it can be seen that this plot follows the trend of gyroscope while maintaining a value indicated by the accelerometer.

3) Walking Test:
For this test, a person walked around carrying the camera, changing the orientation in a random fashion. The file: phase1_final_walking_test.xlsx provides the logged values and plots. In this case, though the variation in states is much more frequent, the Kalman Filter still provides an estimate that strikes balance between gyroscope readings and accelerometer readings.  

Please note that the roll, pitch and yaw are using Euler angle represenatation. This means that the filter would fail when the pitch is around +/-(pi/2) radian. However, this scenario can be avoided for this particular project.

# Authors:
Author1: Syamprasad K Rajagopalan
Email1: syamprasad.rajagopalan@technicalproductsinc.us
Email2: syamkrajagopalan@gmail.com
Author2 (Supervisor): Ryan Mahoney
Email1: ryan.mahoney@technicalproductsinc.us

# Acknowledgement:
Paul Chambers
Email1: paul.chambers@technicalproductsinc.us
