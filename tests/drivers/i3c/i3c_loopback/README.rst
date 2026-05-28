# I3C loopback test

End-to-end test that drives an I3C controller against an I3C target
where both endpoints are real Zephyr I3C drivers wired together on
the bus. Modeled after the SPI loopback test pattern.

## Hardware setup

Two physically separate I3C controllers on the same SoC, with their
SCL and SDA lines externally jumpered together (one jumper for SCL,
one for SDA). One controller runs in controller mode; the other has
``i3c_target_register()`` called on it so it runs in target mode.

Required twister fixture: ``i3c_loopback``

## Required devicetree overlay

The test source uses two devicetree aliases plus a child node on the
controller bus that describes the target's static address and PID.
All other target identity (BCR, DCR, MRL, MWL, supported HDR modes)
is queried at runtime from the target-mode controller via
``i3c_config_get_target()`` so the test never duplicates the truth.

A board overlay must provide:

.. code-block:: dts

   / {
       aliases {
           i3c-loopback-controller = &i3c1;
           i3c-loopback-target = &i3c0;
       };
   };

   &i3c0 {
       status = "okay";
       /* target-mode controller; i3c_target_register attaches at boot */
   };

   &i3c1 {
       status = "okay";
       /* controller-mode controller */

       loopback_target: target@5400abcd00001234 {
           compatible = "zephyr,i3c-loopback-target";
           reg = <0x54 0xabcd 0x00001234>;
       };
   };

Then run::

    west twister -p <board> -T tests/drivers/i3c/i3c_loopback \
        --fixture i3c_loopback

## Test coverage

================================  ====================================
Suite                             What it exercises
================================  ====================================
``i3c_loopback_daa``              RSTDAA, ENTDAA, SETDASA, SETNEWDA,
                                  SETAASA via ``i3c_bus_*`` wrappers;
                                  ``i3c_device_info_get``.
``i3c_loopback_attach``           ``i3c_attach_i3c_device`` / reattach
                                  (after SETNEWDA) / detach lifecycle.
``i3c_loopback_xfer``             Private SDR W/R: 1-byte, multi-byte,
                                  multi-msg with STOP and RESTART;
                                  NBCH; ``i3c_write`` / ``i3c_read`` /
                                  ``i3c_write_read`` / ``i3c_burst_*``
                                  / ``i3c_reg_*`` helpers; SETMWL /
                                  SETMRL limit enforcement.
``i3c_loopback_ccc``              GETBCR, GETDCR, GETPID, GETSTATUS,
                                  GETCAPS, GETMXDS (gated on
                                  ``I3C_BCR_MAX_DATA_SPEED_LIMIT``),
                                  GETMRL, GETMWL, SETMRL/SETMWL
                                  roundtrip, ENEC/DISEC, RSTACT.
``i3c_loopback_configure``        ``i3c_configure_controller`` /
                                  ``i3c_config_get_controller`` /
                                  ``i3c_config_get_target``; SCL
                                  change + roundtrip +
                                  transfer-at-new-speed.
``i3c_loopback_ibi``              TIR raise with and without payload
                                  (gated on
                                  ``I3C_BCR_IBI_REQUEST_CAPABLE``
                                  and ``I3C_BCR_IBI_PAYLOAD_HAS_DATA_BYTE``);
                                  DISEC blocks; ENEC re-enables; HJ
                                  after RSTDAA; ``i3c_ibi_hj_response``
                                  gating.
``i3c_loopback_hdr_ddr``          HDR-DDR write/read (single + multi
                                  word). Suite-level skip if either
                                  side lacks ``I3C_MSG_HDR_DDR`` in
                                  its ``supported_hdr`` bitmask.
``i3c_loopback_target_api``       ``i3c_target_register``/unregister
                                  lifecycle; ``i3c_target_tx_write``
                                  byte-count return.
================================  ====================================

## Explicit non-coverage

- Controller-role handoff (GETACCCR, ``i3c_target_controller_handoff``,
  ``i3c_device_controller_handoff``, ``i3c_ibi_crr_response``) — both
  endpoints would need to be controller-capable, with significant
  orchestration.
- ``i3c_recover_bus`` — hard to provoke a stuck bus deterministically.
- ``i3c_bus_deftgts`` — secondary-controller bookkeeping.
- Vendor CCCs — vendor-specific.
- HDR-TS / HDR-BT — not supported by most controllers.
