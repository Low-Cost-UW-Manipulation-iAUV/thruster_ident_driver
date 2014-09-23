This controller does the thruster identification data acquisiton.
*     It will drive the thruster through the complete 'positive' range.
*     For the inverse a second run is necessary, limitation is the BBB ADC
*     The driver will keep a thrust comand until the force feedback signal is stable.
*     The status will be published on a ros topic, with a flag if it was stable.

*   23/Sept/2014
*///////////////////////////////////////////////////////////////////////
