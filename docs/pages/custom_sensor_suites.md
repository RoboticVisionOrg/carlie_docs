---
sort: 7
---

# Custom Sensor Suites
We understand that you may wish to utilise other sensors, or replace the default sensor suite, with Carlie. You should be able to utilise any sensor that you would on a normal robotic platform as long as it can be mounted to Carlie and does not inhibit movement as well as the power requirements are suitable. 

There are 5V and 6V auxillary power points located on the low-level PCB, see the image below. However, as the connector type can be very sensor specific some soldering/cable creation will be required to access them. The 5V and 6V lines can power up to 2.5A each. With the default Carlie hardware this means approximately an additional 1.5A and 500mA can be loaded onto the 5V and 6V lines respectively.

A 12V power source can be sourced by splicing an intermediate cable to sit between the low-level PCB power management system and the computer, see the image below.  The 12V source can power up to 5A. The TX2 and onboard router can draw up to approximately 1.25A and 500mA each, meaning there is approximately 3A available to power additional components on the 12V line. 