# i3c_emul handoff — Mac → Linux

Live work-in-progress on a full Zephyr I3C bus emulator (`drivers/i3c/i3c_emul.c`) that mirrors `drivers/i2c/i2c_emul.c` but covers the entire `struct i3c_driver_api` surface. Started on macOS where `native_sim` is unavailable; continuing on Linux for full ztest coverage.

## Pickup checklist (Linux side)

```bash
# 1. Pull the branch
cd ~/zephyrproject/zephyr   # or wherever your west workspace lives
git fetch origin i3c-emul-wip
git checkout i3c-emul-wip

# 2. Sanity build
source ../.venv/bin/activate    # or wherever west is installed
west build -p auto -b native_sim samples/hello_world

# 3. Start a fresh Claude session in this directory and feed it:
#    - .handoff/PLAN.md      (the full 7-milestone plan)
#    - .handoff/HANDOFF.md   (this file)
#    Suggested opener prompt: "Pick up the i3c_emul plan from M2.
#    Read .handoff/PLAN.md and .handoff/HANDOFF.md, then start
#    Milestone M2."
```

## Status as of this commit

| Milestone | Description | Status |
|-----------|-------------|--------|
| **M1** | Framework prep: `emul.h` extended, new `i3c_emul.h` | ✅ done — this commit's parent |
| **M2** | Bus driver core (`i3c_emul.c`): xfers, attach/detach, CCCs, DAA | ⬜ next |
| **M3** | IBI support | ⬜ |
| **M4** | Async (`CONFIG_I3C_CALLBACK`) + RTIO | ⬜ |
| **M5** | Target mode + secondary controller handoff | ⬜ |
| **M6** | Reference peripheral emulator (`zephyr,i3c-test-target`) | ⬜ |
| **M7** | ztest suite + twister metadata + build-all variant | ⬜ |

## Files in M1 (already committed)

- `include/zephyr/drivers/emul.h` — added `EMUL_BUS_TYPE_I3C`, `union bus.i3c`, extended `Z_EMUL_BUS` macro family with `_i3c` slot (between `_i2c` and `_espi`), updated all 4 call sites inside `EMUL_DT_DEFINE`.
- `include/zephyr/drivers/i3c_emul.h` — `struct i3c_emul`, `struct i3c_i2c_emul`, `struct i3c_emul_api { xfers, do_ccc, set_dynamic_addr, ibi_enable, ibi_disable, hdr_ddr_xfers }`, registration + IBI injection helpers.

## Files M2 will touch (per PLAN.md §M2)

- `drivers/i3c/i3c_emul.c` (new, ~700 LOC)
- `drivers/i3c/Kconfig.emul` (new)
- `drivers/i3c/Kconfig` (modify — `source "drivers/i3c/Kconfig.emul"`)
- `drivers/i3c/CMakeLists.txt` (modify — `zephyr_library_sources_ifdef(CONFIG_I3C_EMUL i3c_emul.c)`)
- `dts/bindings/i3c/zephyr,i3c-emul-controller.yaml` (new)
- `tests/drivers/i3c/i3c_emul/src/test_core.c` + scaffolding

## Design decisions worth preserving

These came up during M1 design and shouldn't be re-litigated unless something in M2 forces a change:

1. **`static_addr` field name** in `struct i3c_emul` (not `addr`) — chose for readability since i3c targets have both static and dynamic addresses. The `Z_EMUL_BUS` macro positional slot in `EMUL_DT_DEFINE`'s `addr` initializer is `static_addr` accordingly.
2. **`set_dynamic_addr` callback** added to `struct i3c_emul_api` — preferred over overloading `do_ccc` with synthesized SETDASA payloads during DAA. Bus emulator calls this explicitly during DAA / SETDASA / SETNEWDA / RSTDAA.
3. **Bus emulator owns the address slots** (`struct i3c_addr_slots`), not the peripheral. Peripheral just stores its own `dynamic_addr` mirror for `do_ccc` responses (e.g. GETPID).
4. **CCC dispatch model**: broadcast → walk every attached `i3c_emul`, call `do_ccc(target, payload, true)`. Direct → walk `payload->targets.payloads[]`, look up by addr, call `do_ccc(target, payload, false)`.
5. **`mock_api` fault-injection slot** mirrors the i2c_emul pattern (`drivers/i2c/i2c_emul.c:229-234`). Returning `-ENOSYS` from a mock falls through to the real api.
6. **Test target lives under `tests/drivers/i3c/i3c_emul/src/`**, not under `drivers/sensor/<chip>/`. It's a synthetic `zephyr,i3c-test-target` peripheral, not a real chip — keeps the change additive and isolated. Its DT binding goes under the test directory's `dts/bindings/`.
7. **Out of scope for v1** (per user agreement during plan approval): HDR-TSP/TSL/BT (only HDR-DDR is wired through the optional `hdr_ddr_xfers` callback), multi-controller bus arbitration, timing accuracy enforcement, HDR-DDR parity computation, the `forwards` analogue from i2c_emul.

## Verification done on macOS

`native_sim` is Linux-only, so M1 was verified via cross-compiled qemu builds:

```bash
west build -b qemu_cortex_m3 samples/hello_world           # baseline
west build -b qemu_cortex_a9 tests/drivers/sensor/sbs_gauge -- -DCONFIG_EMUL=y
                                                            # exercises EMUL_DT_INST_DEFINE
                                                            # through the i2c arm of Z_EMUL_BUS
```

Both clean. Once on Linux, please re-run the full emul test suite to be sure:

```bash
./scripts/twister -T tests/subsys/emul -p native_sim -v
./scripts/twister -T tests/drivers/i2c/i2c_emul -p native_sim -v
./scripts/twister -T tests/drivers/build_all/i3c -v
```

Any of those failing means M1 needs revisiting before M2 starts.

## Things to watch for in M2

These are the most likely places M2 will surprise you:

- **`i3c_bus_init()` reentrancy**: the i3c subsystem may try to call back into the controller's API during init. Make sure `i3c_emul_init` runs `sys_slist_init` and `emul_init_for_bus` *before* any path that could trigger `attach_i3c_device`. The DT property `disable-bus-init` is a useful escape hatch for tests.
- **DAA address allocation** must respect the `assigned-address` DT property when present. `i3c_addr_slots_next_free_find()` doesn't know about preferences — bus emulator has to check DT first, then fall back to the slot finder.
- **Peripheral doesn't know its own dynamic address** until the controller assigns it. The `set_dynamic_addr` callback is how the bus emulator tells it. Without that, peripheral's GETPID response won't know which dynamic address it's responding from.
- **`i3c_xfers` target lookup**: the `struct i3c_device_desc *target` parameter has both `static_addr` and `dynamic_addr` fields. Match on `dynamic_addr` (which the controller assigned), not `static_addr`.
- **CCC ID range**: `I3C_CCC_BROADCAST_MAX_ID` is 0x7F. Anything ≤ 0x7F is broadcast; anything > 0x7F is direct. There's a couple of CCCs the bus emulator should intercept (RSTDAA → call `set_dynamic_addr(0)` on every peripheral; SETDASA / SETNEWDA / SETAASA → update the address slot mapping after the peripheral acks). See `i3c_ccc.c` for the existing convenience wrappers.

## Key reference points (already vetted, don't re-explore)

- `drivers/i2c/i2c_emul.c:1-341` — full template for the bus driver shape.
- `drivers/spi/spi_emul.c:1-157` — leaner template, useful comparison.
- `include/zephyr/drivers/i3c.h:547-943` — `struct i3c_driver_api`. Every hook here must be implemented (or stubbed with an explicit comment).
- `include/zephyr/drivers/i3c.h:988-1212` — `struct i3c_device_desc` field layout.
- `include/zephyr/drivers/i3c/addresses.h` — `i3c_addr_slots` API, **reuse, don't reimplement**.
- `include/zephyr/drivers/i3c/ccc.h` — every CCC ID + per-CCC payload struct.
- `drivers/i3c/i3c_common.c` (1572 LOC) — `i3c_dev_list_*`, `i3c_dev_attached_*`, bus init helpers. Reuse aggressively.
- `subsys/emul/emul.c::emul_init_for_bus` — registration glue, no changes needed.

## Things NOT in the plan that I noticed during M1

- The base `i3c-controller.yaml` has `bus: [i3c, i2c]` — meaning a child node could be on either bus. For the emul controller, **only `on-bus: i3c` children are routed via `Z_EMUL_BUS`'s `_i3c` arm**. A pure I2C child (with `on-bus: i2c`) would land in the `_i2c` arm and try to use `i2c_emul` infrastructure — which won't work because the parent is an i3c device. M2's `attach_i2c_device` handles legacy I2C-on-I3C *targets* (via `struct i3c_i2c_emul`), but those targets need a binding that says `on-bus: i3c` (it just describes a legacy device on an i3c bus). Double-check this when authoring the test target binding in M6.
- `i3c.h` includes `<zephyr/rtio/rtio.h>` unconditionally. So the test target peripheral emul (M6) will pull in RTIO transitively even if `CONFIG_I3C_RTIO=n`. Not a blocker, just noting.

## Cleanup

When work lands (or branch is abandoned), drop this directory:
```bash
git rm -r .handoff && git commit -m "chore: drop handoff scratch"
```
Or rebase to drop the `.handoff` commit entirely before opening the upstream PR.
