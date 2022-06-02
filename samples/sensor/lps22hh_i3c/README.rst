.. _lps22hh_i3c:

LPS22HH: Temperature and Pressure Monitor (I3C)
###############################################

Overview
********
This sample periodically reads pressure from the LPS22HH MEMS pressure
sensor and displays it on the console.

Requirements
************

This sample uses the LPS22HH sensor controlled using the I3C interface.
It has been tested using the LPS22HH on :ref:`x-nucleo-iks01a3`
connected to the I3C header on :ref:`mimxrt685_evk`.

References
**********

- LPS22HH: http://www.st.com/en/mems-and-sensors/lps22hh.html

Building and Running
********************

This project outputs sensor data to the console. It requires an LPS22HH
sensor (for example, the one on :ref:`x-nucleo-iks01a3`).

Building on mimxrt685_evk_cm33 board
====================================

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/lps22hh_i3c
   :host-os: unix
   :board: mimxrt685_evk_cm33
   :goals: build
   :compact:


Board Preparations
==================

mimxrt685_evk_cm33
------------------

On the board :ref:`mimxrt685_evk`, the I3C pins are exposed on the J18
header, where:

  * SCL is on pin 1
  * SDA is on pin 2
  * Internal pull-up is on pin 3 (which is connected to pin 2 already)
  * Ground is on pin4

Using Generic LPS22HH
^^^^^^^^^^^^^^^^^^^^^

A LPS22HH sensor needs to be connected to this header. For example,
the evaluation board STEVAL-MKI196V1 can be used. This needs to be
prepared so that the LPS22HH sensor has address 0x5D (i.e. 0xBA,
left-shifed).

Using LPS22HH on X-NUCLEO-IKS01A3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The LPS22HH on :ref:`x-nucleo-iks01a3` can be used by plugging
the shield into the onboard Arduino header. Two jumper wires
are needed to connect the J18 header to the SCL/SDA pins on
the shield (J2, which is the I2C2 on shield). Do NOT connect pin 3
between these two headers. Note that with this setup, the I2C
controller must be disabled (by making sure ``CONFIG_I2C=n``) such that
only the I3C controller is driving the pins.

Sample Output
=============

.. code-block:: console

   Configured for triggered collection at 1 Hz
   Observation: 1
   Pressure: 97.474 kPa
   Temperature: 22.19 C
   Observation: 2
   Pressure: 97.466 kPa
   Temperature: 22.21 C
   Observation: 3
   Pressure: 97.473 kPa
   Temperature: 22.21 C
   Observation: 4
   Pressure: 97.455 kPa
   Temperature: 22.21 C

   <repeats endlessly every second>
