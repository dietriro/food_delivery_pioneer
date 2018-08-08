# Food Delivery Pioneer

## Description
This repository includes code for an autonomous food delivery robot implemented for the final project of class ROB551 (Mobile Robots) at Oregon State University. The goal of this project was to offer a service where a student can order something through an online platform, an employee packs the order ontop of the robot and sends it to the student (via a website as well). We created a webservice for both the employee and the customer using the rosbridge-suite (wiki.ros.org/rosbridge_suite). Additionaly the standard map-server was extended in order to be able to switch between different maps online. For traveling between multiple floors, the robot is equipped with an arm which is able to push elevator buttons and a camera for the detection of the buttons.

For a more in depth description of the project see https://sites.google.com/oregonstate.edu/fooddelpi/home.

## Packages

<dl>
  <dt>foodelpi_navigation</dt>
  <dd>Contains robot specific maps, configs and launch files for running the navigation stack.</dd>

  <dt>foodelpi_simulator</dt>
  <dd>Contains the maps and a launch file for running stage using the real-world setup.</dd>
  
  <dt>foodelpi_webservice</dt>
  <dd>Contains code for running a webservice for both the customer and the service employee.</dd>
  
  <dt>multi_map_server</dt>
  <dd>The standard ROS map-server with the additional functionality of switching online between different maps.</dd>
</dl>
