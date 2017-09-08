Code samples for Boldport's IxapandO
====================================

IxpandO
-------

IxpandO_ is a Boldport's project, based on the MCP23017_, which is a
'16-Bit I/O Expander with Serial Interface (I2C)'.

Board configuration for my code
-------------------------------

IxpandO A port is used as output (with LEDs)

IxpandO B port is used as input (with switchs)

The IXO class is used to communicate with IxpandO over the Wire library,
it should probably be in a common header file,
but I'm not sure how to do that in the arduino environment.


arduino_loop
------------

Drive Boldport's IxpandO from arduino Uno.

.. csv-table:: Connections
    :header: "Arduino", "IxpandO"

    "5V", "VDD"
    "GND", "GND"
    "A0", "RSTn"
    "A4 (SDA)", "SDA"
    "A5 (SCL)", "SCK"


The arduino code is pretty straight forward,
After initialization, it will read port B, reverse the bits,
and write it to port A over and over again.

arduino_interrupts
------------------

Drive Boldport's IxpandO from arduino Uno, but with interrupts.

.. csv-table:: Connections
    :header: "Arduino", "IxpandO"

    "5V", "VDD"
    "GND", "GND"
    "A0", "RSTn"
    "A4 (SDA)", "SDA"
    "A5 (SCL)", "SCK"
    "3 (PWN)", "INT B"

In this example,
reading port B and writing to port A is not done each time,
only when INTB interrupt occurs.

.. _IxpandO: https://www.boldport.com/products/ixpando/
.. _MCP23017: https://www.microchip.com/wwwproducts/en/MCP23017