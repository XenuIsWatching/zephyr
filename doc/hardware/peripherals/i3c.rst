.. _i3c_api:

I3C
###

Overview
********

I3C (Improved Inter-Integrated Circuit) is a two-signal shared
peripheral interface bus.  Devices on the bus can operate in
two roles: as a "controller" that initiates transactions and
controls the clock, or as a "target" that responds to transaction
commands.

Currently, the API is based on `I3C Specification`_ version 1.1.1.

.. _i3c-controller-api:

I3C Controller API
==================

Zephyr's I3C controller API is used when an I3C controller controls
the bus, in particularly the start and stop conditions and the clock.
This is the most common mode, used to interact with I3C target
devices such as sensors.

Due to the nature of the I3C, there are devices on the bus where
they do not have addresses when powered on. Therefore, an additional
dynamic address assignment needs to be carried out by the I3C
controller. Here is a list of generic steps for initializing the I3C
controller by its device driver:

#. Initialize the data structure of the I3C controller device
   driver instance. This initialization function is provided to
   :c:macro:`I3C_DEVICE_DT_INST_DEFINE`.

   * Prepare the data struct to store device list when
     :c:func:`i3c_device_register` is called.

     * The :c:struct:`i3c_dev_list` is a generic structure to
       store the device list. If this is being used,
       this struct needs to be initialized by calling
       :c:func:`i3c_dev_list_init`.

#. The target devices on the same bus need to call
   :c:func:`i3c_device_register` so the controller device driver
   instance is aware of these target devices. This tells
   the controller device driver instance how to assign addresses
   (static vs. dynamic).

   * For I2C devices on the bus, they need to call
     :c:func:`i3c_i2c_device_register` to announce their presence.

   * Note that the controller should not assign addresses to
     devices if their device drivers have not registered their
     presence. This is due to the API requiring a device descriptor
     struct to be supplied, and not-yet-registered devices do not
     have associated descriptors.

   * An extra initialization function can be provided to
     :c:macro:`I3C_DEVICE_REGISTER_INIT` to aid in device
     registration. This function needs to call
     :c:func:`i3c_device_register` or
     :c:func:`i3c_i2c_device_register` according to device type.

#. The bus initialization function provided to
   :c:macro:`I3C_BUS_INIT` is called to initialize the bus.

   #. Initialize the hardware, including but not limited to:

      * Setup pin mux and directions.

      * Setup the clock for the controller.

      * Power on the hardware.

      * Configure the hardware (e.g. SCL clock frequency).

   #. Do ``RSTDAA`` to reset dynamic addresses of connected devices.
      If any connected devices have already been assigned an address,
      the bookkeeping data structures do not have records of these,
      for example, at power-on. So it is a good idea to reset and
      assign them new addresses.

   #. Do ``SETDASA`` to use static addresses as dynamic address
      if so desired.

      * ``SETAASA`` may not be supported for all connected devices
        to assign static addresses as dynamic addresses.

      * BCR and DCR need to be obtained separately to populate
        the relevant fields in the I3C target device descriptor
        struct.

   #. Do ``ENTDAA`` to start dynamic address assignment.

      * If there is a device waiting for address, it will send
        its Provisioned ID, BCR, and DCR back. Match the received
        Provisioned ID to the list of registered I3C devices.

        * If there is a match, assign an address (either from
          the stated static address if ``SETDASA`` has not been
          done, or use a free address).

          * Also, set the BCR and DCR fields in the device descriptor
            struct.

        * If there is no match, depending on policy, it can be
          assigned a free address, or the device driver can stop
          the assignment process and errors out.

          * Note that the I3C API requires device descriptor to
            function. A device without a device descriptor cannot be
            accessed through the API.

      * This step can be skipped if there is no connected devices
        requiring DAA.

   #. These are optional but highly recommended:

      * Do ``GETMRL`` and ``GETMWL`` to get maximum read/write
        length.

      * Do ``GETMXDS`` to get maximum read/write speed and maximum
        read turnaround time.

In-Band Interrupt (IBI)
-----------------------

If a target device can generate In-Band Interrupt (IBI),
the controller needs to be made aware of it.

* :c:func:`i3c_ibi_slot_request` to request/allocate an IBI slot
  in the controller.

  * This sets up the IBI slot so the controller can recognize
    incoming IBI from target devices.

  * This only programs the IBI slot, and the IBI is not active.
    Call :c:func:`i3c_ibi_enable` to activate the slot.

  * Note that there are usually limited IBI slots on
    the controller so this operation may fail.

* :c:func:`i3c_ibi_slot_free` to free a previosly allocated IBI
  slot. Use this when IBI is no longer needed to recognized by
  the controller.

* :c:func:`i3c_ibi_enable` enables the controller to raise event(s)
  when there is an incoming IBI.

* :c:func:`i3c_ibi_disable` tells controller not to raise any
  event(s) where there is incoming IBIs.

Configuration Options
*********************

Related configuration options:

* :kconfig:option:`CONFIG_I3C`
* :kconfig:option:`CONFIG_I3C_USE_GROUP_ADDR`
* :kconfig:option:`CONFIG_I3C_USE_IBI`
* :kconfig:option:`CONFIG_I3C_IBI_MAX_PAYLOAD_SIZE`

API Reference
*************

.. doxygengroup:: i3c_controller_interface
.. doxygengroup:: i3c_ccc
.. doxygengroup:: i3c_addresses

Links
*****

.. _I3C Specification: https://www.mipi.org/specifications/i3c-sensor-specification
