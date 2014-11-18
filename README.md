thruster_ident_driver
=======================

This controller does the thruster identification data acquisiton.
*     It will drive the thruster through the complete 'positive' range.
*     For the inverse a second run is necessary, limitation is the BBB ADC
*     The driver will keep a thrust comand until the force feedback signal is stable.
*     The status will be published on a ros topic, with a flag if it was stable.

*   23/Sept/2014
*///////////////////////////////////////////////////////////////////////

You start it after loading the yaml file:

rosparam load .../src/thruster_ident_driver/thruster_ident_param.yaml

rosrun controller_manager spawner /ros_control_iso/thruster_ident_driver


It will sit and wait for the calibration data, so you should star the thruster_ident_adc node as well.