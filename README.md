# TesLorean-Battery-Controller
Arduino code to control the Chevy Spark EV 2014 A123 battery module and and monitor the BMS

The TesLorean is fitted with a Chevy Spark EV 2014 battery pack containing modules made by A123.

The Pack has two sets of B+B- high voltage lines, one for charging (no pre-charge protection) and one for drive systems (pre-charge protected).  There are two data/control low voltage ports.  One port connects to the BMS and output battery cell status, balancing information, and temperatures.  The other port has a number of lines dedicated to activating (with 12v) high voltage contactors.

The TesLorean Battery Controller will accept instructions to activate the high voltage contactors, and will also monitor cell statuses and temperature.  The controller will send summary battery status information and warning messages should any individual cell go into over or under voltage.  The battery controller will disconnect the pack (open the contactors) if the pack voltage falls too long - risking long-term damage.

