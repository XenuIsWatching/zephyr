.. _lsm6dso_i3c:

LSM6DSO: IMU Sensor Monitor (I3C)
#################################

Overview
********
This sample sets the date rate of LSM6DSO accelerometer and gyroscope to
12.5Hz and enables a trigger on data ready. It displays on the console
the values for accelerometer and gyroscope.

Requirements
************

This sample uses the LSM6DSO sensor controlled using the I3C interface.
It has been tested using the LSM6DSO on :ref:`x-nucleo-iks01a3`
connected to the I3C header on :ref:`mimxrt685_evk`.

References
**********

- LSM6DSO http://www.st.com/en/mems-and-sensors/lsm6dso.html

Building and Running
********************

This project outputs sensor data to the console. It requires an LSM6DSO
sensor (for example, the one on :ref:`x-nucleo-iks01a3`).

Building on mimxrt685_evk_cm33 board
====================================

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/lsm6dso
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

Using Generic LSM6DSO
^^^^^^^^^^^^^^^^^^^^^

A LSM6DSO sensor needs to be connected to this header. For example,
the evaluation board STEVAL-MKI196V1 can be used. This needs to be
prepared so that the LSM6DSO sensor has address 0x6B (i.e. 0xD6,
left-shifed).

Using LSM6DSO on X-NUCLEO-IKS01A3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The LSM6DSO on :ref:`x-nucleo-iks01a3` can be used by plugging
the shield into the onboard Arduino header. Two jumper wires
are needed to connect the J18 header to the SCL/SDA pins on
the shield (J2, which is the I2C2 on shield). Do NOT connect pin 3
between these two headers. Note that with this setup, the I2C
controller must be disabled (by making sure ``CONFIG_I2C=n``) such that
only the I3C controller is driving the pins.

Sample Output
=============

.. code-block:: console

    Testing LSM6DSO sensor in trigger mode.

    accel x:-0.650847 ms/2 y:-5.300102 ms/2 z:-8.163114 ms/2
    gyro x:-0.167835 dps y:-0.063377 dps z:0.002367 dps
    trig_cnt:1

    accel x:0.341575 ms/2 y:5.209773 ms/2 z:-7.938787 ms/2
    gyro x:-0.034284 dps y:-0.004428 dps z:-0.003512 dps
    trig_cnt:2

    <repeats endlessly>
