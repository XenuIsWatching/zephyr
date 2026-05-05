# Plan: `i3c_emul.c` — full I3C bus emulator for Zephyr

**Goal**: Add a Zephyr emulated I3C controller (`zephyr,i3c-emul-controller`) that mirrors `i2c_emul.c` in role but covers the entire `struct i3c_driver_api`: private xfers, CCCs (broadcast + direct), DAA, IBI, async/callback path, RTIO submit, and target/secondary-controller handoff. Ship with a reference peripheral emulator and a ztest suite so consumers (sensors, the i3c subsystem, application code) can be tested on `native_sim` without real silicon.

**Architecture**: Same parent/child split as I2C — one bus emulator (this driver) routes per-target via the existing `struct emul` plumbing (`include/zephyr/drivers/emul.h:82-104, 142-160`) to peripheral emulators that implement `struct i3c_emul_api`. Reuses `subsys/emul/emul.c::emul_init_for_bus` and the `EMUL_DT_DEFINE` macro family unchanged in spirit; adds an i3c arm to the `enum emul_bus_type` / `union bus` / `Z_EMUL_BUS` triplet.

**Tech stack**: C (Zephyr coding style + DEVICE_API + DT_INST_FOREACH macros), devicetree YAML bindings, Kconfig, CMake, ztest. Target build for `native_sim`.

---

## Context

`drivers/i2c/i2c_emul.c` is the proven pattern for testing I2C-side code without hardware: a bus driver that dispatches `i2c_transfer()` to per-target peripheral emulators registered through `EMUL_DT_DEFINE`. Tests use this in two layers — a transport layer (the peripheral emul fakes register reads/writes) and an optional sideband **backend API** (`struct emul.backend_api`) that lets tests poke values from outside the bus model. A `mock_api` slot also exists for fault injection (`drivers/i2c/i2c_emul.c:229-234`).

I3C has an order-of-magnitude richer API than I2C — DAA (dynamic address assignment), CCCs (broadcast + direct), IBI (in-band interrupts), HDR-DDR transfers, mixed I2C-on-I3C targets, secondary-controller handoff, async callbacks (`CONFIG_I3C_CALLBACK`) and RTIO submit (`CONFIG_I3C_RTIO`). Today the only "fake" i3c driver in tree is `drivers/i3c/i3c_test.c` (47 lines, every op returns `-ENOTSUP`); no functional i3c emulator exists, which blocks i3c-side test coverage entirely.

This plan adds a **functional** emulator covering the full controller API surface, on the same model as i2c_emul. Scope chosen by user: **full controller API** (xfers + CCCs + DAA + IBI + HDR + async + RTIO + target/handoff). Target mode is included; secondary-controller handoff covered by storing role and acknowledging the hooks (deep handoff state machine is left out — emulator just records the request and notifies via callback).

---

## Deliverables (file inventory)

### New files

- `include/zephyr/drivers/i3c_emul.h` — public header for the bus↔peripheral contract (parallel to `include/zephyr/drivers/i2c_emul.h`).
- `drivers/i3c/i3c_emul.c` — the bus driver (estimated ~700-900 LOC).
- `drivers/i3c/Kconfig.emul` — `CONFIG_I3C_EMUL` and any sub-options (e.g. forwarding, max emulators).
- `dts/bindings/i3c/zephyr,i3c-emul-controller.yaml` — DT binding.
- `drivers/sensor/<picked_chip>/<chip>_emul.c` **or** `tests/drivers/i3c/i3c_emul/src/test_target_emul.c` — a reference peripheral emulator. Recommended: a synthetic `zephyr,i3c-test-target` peripheral that lives entirely under `tests/` rather than touching a real sensor driver (keeps the change additive and isolated).
- `dts/bindings/i3c/zephyr,i3c-test-target.yaml` (in `tests/.../boards/` overlay or under `dts/bindings/test/`) — DT binding for the test target.
- `tests/drivers/i3c/i3c_emul/` — full ztest project: `CMakeLists.txt`, `prj.conf`, `boards/native_sim.overlay`, `src/main.c`, plus per-feature test files.

### Modified files

- `include/zephyr/drivers/emul.h` — add `EMUL_BUS_TYPE_I3C`, add `struct i3c_emul *i3c` to `union bus`, extend `Z_EMUL_BUS()` with the i3c branch, extend `EMUL_DT_DEFINE` invocations of `Z_EMUL_BUS` to include i3c (5 places: lines 120-126, 143, 147, 154-156, 157). Also add `#include <zephyr/drivers/i3c_emul.h>` (header dance: must forward-declare or use a guarded include order).
- `drivers/i3c/CMakeLists.txt` — add `zephyr_library_sources_ifdef(CONFIG_I3C_EMUL i3c_emul.c)`.
- `drivers/i3c/Kconfig` — `source "drivers/i3c/Kconfig.emul"` near the other driver-specific Kconfig sources (around line 223-233).

---

## Critical reference points (read these first when executing)

- `drivers/i2c/i2c_emul.c:1-341` — full template for the bus driver shape.
- `drivers/spi/spi_emul.c:1-157` — second template; useful for the leaner "no target mode" core.
- `include/zephyr/drivers/i2c_emul.h:38-85` — `struct i2c_emul` + `struct i2c_emul_api` shape.
- `include/zephyr/drivers/emul.h:82-160` — `struct emul`, `EMUL_DT_DEFINE`, `Z_EMUL_BUS` (the macros to extend).
- `subsys/emul/emul.c` (esp. `emul_init_for_bus`, `emul_get_binding`) — registration glue, **no changes needed** here.
- `include/zephyr/drivers/i3c.h:547-943` — `struct i3c_driver_api`; every hook in this vtable must be implemented (or `-ENOSYS`/`-ENOTSUP` documented).
- `include/zephyr/drivers/i3c.h:988-1212` — `struct i3c_device_desc`, `struct i3c_i2c_device_desc` field-by-field (`pid`, `static_addr`, `dynamic_addr`, `bcr`, `dcr`, `data_speed`, `data_length`, `getcaps`, `crcaps`, `controller_priv`, `ibi_cb`).
- `include/zephyr/drivers/i3c/addresses.h` — full `i3c_addr_slots` API; **reuse, do not reimplement**.
- `include/zephyr/drivers/i3c/ccc.h:1-2190` — every CCC ID + per-CCC payload struct.
- `include/zephyr/drivers/i3c/ibi.h` + `drivers/i3c/i3c_ibi_workq.c` — IBI types + workqueue path.
- `drivers/i3c/i3c_common.c` (1572 LOC) — bus init, DAA helpers, `i3c_dev_list_*`, `i3c_dev_attached_*`. Reuse aggressively.
- `drivers/i3c/i3c_ccc.c` (964 LOC) — `i3c_ccc_do_*` helpers; the emulator's `do_ccc` lives "below" these — they're the *callers* into `api->do_ccc`.
- `drivers/i3c/i3c_rtio.c` + `drivers/i3c/i3c_rtio_default.c` — for the RTIO `iodev_submit` plug-in.
- `dts/bindings/i2c/zephyr,i2c-emul-controller.yaml` — binding template.
- `dts/bindings/i3c/i3c-controller.yaml` — base binding to `include:`.

---

## Execution order

| Group | Steps | Parallelizable? | Notes |
|-------|-------|-----------------|-------|
| 1 | M1 | No | Framework header changes — must land before any peripheral can be built. |
| 2 | M2 | No | Bus driver MVP (xfers + CCCs + DAA + attach/detach), the load-bearing change. |
| 3 | M3, M4 | M3 and M4 yes (independent files within driver) | IBI + async/RTIO. Built on M2. |
| 4 | M5 | No | Target mode + handoff — separate code path but still in i3c_emul.c. |
| 5 | M6 | No | Reference peripheral emulator. Needs M2-M5 stable. |
| 6 | M7 | No | Tests. Final verification gate. |

Within each milestone, follow TDD: write/extend a ztest first, watch it fail, implement, watch it pass, then commit.

---

## Milestone M1 — Framework prep

**Goal**: extend `enum emul_bus_type` / `union bus` / `Z_EMUL_BUS` so that an i3c child of an emulated bus selects the i3c arm, and create the `struct i3c_emul` / `struct i3c_emul_api` header so M2 can reference it.

### M1.1 — Add forward decl + new bus type
**File**: `include/zephyr/drivers/emul.h`

- Add `struct i3c_emul;` forward declaration near the existing `struct emul;` forward decl (around line 22).
- Add `#include <zephyr/drivers/i3c_emul.h>` to the include block (around line 28-31). Place it **after** the i2c/spi includes to match alphabetical/dependency order.
- Add `EMUL_BUS_TYPE_I3C,` to `enum emul_bus_type` (lines 37-44). Place between `EMUL_BUS_TYPE_I2C` and `EMUL_BUS_TYPE_ESPI` to keep grouping sensible.
- Add `struct i3c_emul *i3c;` to the `union bus` (lines 94-101).

### M1.2 — Extend `Z_EMUL_BUS` macro family
**File**: `include/zephyr/drivers/emul.h`

`Z_EMUL_BUS` currently takes one slot per bus type (`_i2c, _espi, _spi, _mspi, _uart, _none`). Add an `_i3c` slot at the appropriate position (line 120-126). Then update **every call site** of `Z_EMUL_BUS` inside `EMUL_DT_DEFINE` (line 143, 147, 154, 157) to pass the new i3c argument:
- struct type slot: `i3c_emul`
- addr-field slot: `addr` (i3c targets have static addr in `reg[0]`, so the same `DT_REG_ADDR` macro works for the static address; the dynamic address is set later by DAA)
- bus-type enum slot: `EMUL_BUS_TYPE_I3C`
- union member slot: `i3c`

Also extend the `DT_ON_BUS(node_id, i3c)` check inside `Z_EMUL_BUS`'s `COND_CASE_1` chain.

### M1.3 — Create `i3c_emul.h`
**File**: `include/zephyr/drivers/i3c_emul.h` (new)

Contents:
```c
struct i3c_emul {
    sys_snode_t node;
    const struct emul *target;
    const struct i3c_emul_api *api;
    struct i3c_emul_api *mock_api;
    /* Static address from DT reg[0]. 0 if DAA-only. */
    uint8_t static_addr;
    /* Dynamic address; written by emulator's do_daa or SETDASA handler. */
    uint8_t dynamic_addr;
    /* 48-bit Provisioned ID. */
    uint64_t pid;
    uint8_t bcr;
    uint8_t dcr;
};

struct i3c_i2c_emul {  /* legacy I2C target on i3c bus */
    sys_snode_t node;
    const struct emul *target;
    const struct i3c_emul_api *api;  /* same vtable, only xfers/legacy CCCs valid */
    uint16_t addr;
    uint8_t lvr;
};

typedef int (*i3c_emul_xfers_t)(const struct emul *target,
                                struct i3c_msg *msgs, uint8_t num_msgs);
typedef int (*i3c_emul_do_ccc_t)(const struct emul *target,
                                 struct i3c_ccc_payload *payload,
                                 bool is_broadcast);  /* dispatched per-target */
typedef int (*i3c_emul_ibi_enable_t)(const struct emul *target);
typedef int (*i3c_emul_ibi_disable_t)(const struct emul *target);

struct i3c_emul_api {
    i3c_emul_xfers_t      xfers;
    i3c_emul_do_ccc_t     do_ccc;
    i3c_emul_ibi_enable_t ibi_enable;
    i3c_emul_ibi_disable_t ibi_disable;
    /* Optional: HDR-DDR distinct from xfers; if NULL, bus emulator falls
       back to xfers with msg->hdr_mode set. */
    i3c_emul_xfers_t      hdr_ddr_xfers;
};

int i3c_emul_register(const struct device *dev, struct i3c_emul *emul);
int i3c_emul_i2c_register(const struct device *dev, struct i3c_i2c_emul *emul);

/* Used by peripheral emulators to inject IBIs into the bus.
   Bus emulator looks up the registered i3c_device_desc by dynamic_addr,
   invokes its ibi_cb, and ACKs/NACKs based on enable state. */
int i3c_emul_target_raise_ibi(const struct emul *target,
                              uint8_t *payload, uint8_t payload_len);
int i3c_emul_target_raise_hj(const struct emul *target);
int i3c_emul_target_raise_crr(const struct emul *target);
```

### M1.4 — Build smoke test
After M1.1–M1.3, build `samples/hello_world` for `native_sim` to confirm no regressions (`west build -b native_sim samples/hello_world`). The header changes are macro-heavy — ensure no existing emul-using driver fails to compile.

**M1 commit**: "drivers: emul: add i3c bus type and i3c_emul.h scaffolding"

---

## Milestone M2 — Bus driver core (xfers, attach/detach, CCCs, DAA)

**Goal**: a driver with `compatible = "zephyr,i3c-emul-controller"` that completes `i3c_bus_init()` against children declared with `compatible = "zephyr,i3c-test-target"`, runs DAA, and routes private xfers + CCCs to the right peripheral emul.

### M2.1 — Devicetree binding
**File**: `dts/bindings/i3c/zephyr,i3c-emul-controller.yaml` (new)
```yaml
description: Zephyr I3C Emulation controller
compatible: "zephyr,i3c-emul-controller"
include: i3c-controller.yaml
properties:
  reg:
    required: true
```
(`i3c-controller.yaml` already mandates `#address-cells = 3`, `#size-cells = 0`, and exposes `i3c-scl-hz`, `i2c-scl-hz`, `primary-controller-da`, etc.)

### M2.2 — Kconfig
**File**: `drivers/i3c/Kconfig.emul` (new)
```kconfig
config I3C_EMUL
    bool "I3C emulator"
    default y
    depends on DT_HAS_ZEPHYR_I3C_EMUL_CONTROLLER_ENABLED
    depends on EMUL
    help
      Fake I3C bus controller that dispatches to peripheral emulators
      registered via EMUL_DT_DEFINE. Used for testing I3C drivers and
      higher-level subsystems on native_sim without real hardware.
```
**File**: `drivers/i3c/Kconfig` — add `source "drivers/i3c/Kconfig.emul"` near the other driver-specific sources (alongside `Kconfig.cdns`, `Kconfig.test`, etc., around line 223-233).

### M2.3 — CMake hook
**File**: `drivers/i3c/CMakeLists.txt` — add `zephyr_library_sources_ifdef(CONFIG_I3C_EMUL i3c_emul.c)`.

### M2.4 — Driver skeleton
**File**: `drivers/i3c/i3c_emul.c` (new)

Mirrors `drivers/i2c/i2c_emul.c:1-67` and `:244-341`. Includes:
- `#define DT_DRV_COMPAT zephyr_i3c_emul_controller`
- `LOG_MODULE_REGISTER(i3c_emul_ctlr);`
- `struct i3c_emul_data` holding two `sys_slist_t` (one for `i3c_emul`, one for `i3c_i2c_emul`), the `struct i3c_addr_slots` slots, the `struct i3c_config_controller` config, and `struct i3c_dev_attached_list attached`.
- `struct i3c_emul_config` holding `struct i3c_dev_list` (the static DT-derived list of children, populated in the per-instance macro), `struct emul_list_for_bus emul_list`, and the device's `i3c_scl_hz`/`i2c_scl_hz`.
- `i3c_emul_find_by_dyn(dev, addr)` helper (mirrors `i2c_emul_find`).
- `i3c_emul_find_by_pid(dev, pid)` helper for `i3c_device_find`.
- `i3c_emul_init()` that calls `sys_slist_init` x2, `i3c_addr_slots_init(dev)`, `emul_init_for_bus(dev)`, then optionally `i3c_bus_init(dev, &cfg->dev_list)` if `disable-bus-init` is not set.
- `i3c_emul_register()` and `i3c_emul_i2c_register()` (called from peripheral emul init).
- The per-instance macro and `DT_INST_FOREACH_STATUS_OKAY(I3C_EMUL_INIT)`.

Use `I3C_DEVICE_DT_INST_DEFINE` (the i3c subsystem's own DEVICE-define wrapper — check `include/zephyr/drivers/i3c.h` for the macro). If no such wrapper exists, use `DEVICE_DT_INST_DEFINE` with `POST_KERNEL` priority and `CONFIG_I3C_CONTROLLER_INIT_PRIORITY`.

### M2.5 — `configure` / `config_get` / `recover_bus`
Trivial: store/return `struct i3c_config_controller`; `recover_bus` returns 0.

### M2.6 — `attach_i3c_device` / `reattach_i3c_device` / `detach_i3c_device`
Each takes a `struct i3c_device_desc *target`. Implementation:
- `attach`: walk emul list to find peripheral by static address or PID match (PID match needed when `static_addr == 0`); store `target` pointer in the matching `struct i3c_emul` so `i3c_xfers` can map back. Mark the slot in `i3c_addr_slots` if a dynamic address is already assigned.
- `reattach`: update slot mapping for new dynamic address.
- `detach`: clear slot.
Reuse helpers from `drivers/i3c/i3c_common.c` where available (e.g. `i3c_dev_list_daa_addr_helper`).

Also `attach_i2c_device` / `detach_i2c_device` — same shape against the I2C list.

### M2.7 — `do_daa`
Walk every `struct i3c_emul` whose `target` is set but `dynamic_addr == 0`. For each, pick a free address via `i3c_addr_slots_next_free_find()` (respecting any `assigned-address` DT property if present), assign it to `emul->dynamic_addr` *and* to the matching `i3c_device_desc->dynamic_addr`, mark the slot `I3C_ADDR_SLOT_STATUS_I3C_DEV`. The peripheral emul also needs its dynamic address — call its `do_ccc` with a synthesized `I3C_CCC_SETDASA`/`SETNEWDA` payload (cleaner: define a new `struct i3c_emul_api.set_dynamic_addr` callback called explicitly by the bus emul during DAA — preferred since avoids overloading `do_ccc`).

### M2.8 — `do_ccc`
Two paths based on `payload->ccc.id`:
- **Broadcast** (`id <= I3C_CCC_BROADCAST_MAX_ID`, i.e. `0x00..0x7F`): walk all `i3c_emul` entries with `dynamic_addr != 0` (or always, depending on CCC), call each one's `api->do_ccc(target, payload, true)`. Aggregate errors (first non-zero wins, or NACK on any).
- **Direct**: walk `payload->targets.payloads[]`, look up each by `target_payload->addr`, call its `api->do_ccc(target, payload, false)` with the per-target slot in `payload->targets.payloads[i]` accessible (peripheral emul fills `data` and `num_xfer`).

Document the supported CCC list in the driver header (CCCs the *bus emulator itself* must intercept rather than forward to peripherals): `RSTDAA`, `ENTDAA` (handled via `do_daa` instead), `SETDASA`, `SETNEWDA`, `SETAASA` should also update the bus emulator's `i3c_addr_slots` and the peripheral's `dynamic_addr` fields after the peripheral's `do_ccc` succeeds.

### M2.9 — `i3c_xfers`
Match on `target->dynamic_addr` (the `struct i3c_device_desc` passed in). Find the matching `i3c_emul`; call its `api->xfers`. Honor `mock_api` slot exactly like `i2c_emul_transfer` (`drivers/i2c/i2c_emul.c:229-234`).

### M2.10 — `i3c_device_find`
Walk attached list comparing `i3c_device_id` (which has both PID and static-addr matching modes — check the `enum` in `include/zephyr/drivers/i3c.h` near line 1290).

### M2.11 — Tests for M2 (write *first*, then implement)
**File**: `tests/drivers/i3c/i3c_emul/src/test_core.c` (new)

Cases to write before M2.4-M2.10:
- `test_attach_detach_basic` — attach two devices, detach one, verify list state.
- `test_do_daa_assigns_addresses` — declare 3 children with no static addr, run `i3c_do_daa()`, assert all 3 got distinct dynamic addresses in the legal range.
- `test_do_daa_respects_assigned_address` — child with `assigned-address = <0x12>;`, assert it gets exactly 0x12.
- `test_setdasa_static_to_dynamic` — child with `reg = <0x55 0 0>;`, run `i3c_ccc_do_setdasa()`, assert peripheral's dynamic_addr == 0x55.
- `test_xfers_routes_by_dynamic_addr` — write known bytes via `i3c_write()`, peripheral emul records them, assert match.
- `test_ccc_getpid_round_trip` — broadcast `GETPID`, assert peripheral's PID is reported back.
- `test_mock_api_returns_eio` — install a mock_api that returns `-EIO` on `xfers`, verify `i3c_write()` propagates `-EIO`.

**M2 commit (or one per major hook if preferred)**: "drivers: i3c: add i3c_emul controller (xfers, ccc, daa)"

---

## Milestone M3 — IBI

### M3.1 — Implement `ibi_enable` / `ibi_disable`
Look up peripheral by `target->dynamic_addr`, set/clear an `enabled` flag in the bus emulator's per-emul state (independent of the peripheral's own flag for testability), then call `api->ibi_enable`/`disable` if non-NULL.

### M3.2 — Implement `i3c_emul_target_raise_ibi`
Called from peripheral emul code (test-side or backend) to inject an IBI. Walk the bus emul's list of `i3c_emul`, find the caller's entry, look up the corresponding `i3c_device_desc` from the attached list, **if** IBI is enabled, deliver via either:
- direct callback: `desc->ibi_cb(desc, &payload)` (used when `CONFIG_I3C_IBI_WORKQUEUE` is off), or
- workqueue: enqueue via the `i3c_ibi_workq` API (`drivers/i3c/i3c_ibi_workq.c`).

### M3.3 — Implement `ibi_hj_response` and `ibi_crr_response`
Store the ack/nack state in `i3c_emul_data`. `i3c_emul_target_raise_hj` checks this before delivering a hot-join.

### M3.4 — Tests
**File**: `tests/drivers/i3c/i3c_emul/src/test_ibi.c` (new)
- `test_ibi_disabled_drops` — raise IBI without enabling, assert no callback.
- `test_ibi_enabled_delivers_payload` — enable, raise with payload, assert callback fires with same bytes.
- `test_ibi_workq_path` — same as above with `CONFIG_I3C_IBI_WORKQUEUE=y`.
- `test_hj_nack` — `ibi_hj_response(false)`, raise HJ, assert no notification.
- `test_hj_ack_attaches_device` — ack HJ, assert device gets address via subsequent DAA.

**M3 commit**: "drivers: i3c: i3c_emul IBI support (raise/enable/disable/hj/crr)"

---

## Milestone M4 — Async (`CONFIG_I3C_CALLBACK`) and RTIO (`CONFIG_I3C_RTIO`)

### M4.1 — `do_ccc_cb` and `i3c_xfers_cb`
Wrap the synchronous M2.8 / M2.9 implementations: run them inline, then invoke the `i3c_callback_t cb` with the result. Guarded by `#ifdef CONFIG_I3C_CALLBACK`.

### M4.2 — `iodev_submit`
Plug into the i3c RTIO default: `.iodev_submit = i3c_iodev_submit_fallback` (verify the actual name in `drivers/i3c/i3c_rtio_default.c`). Guarded by `#ifdef CONFIG_I3C_RTIO`.

### M4.3 — Tests
**File**: `tests/drivers/i3c/i3c_emul/src/test_async.c` (new)
- `test_xfers_cb_fires_on_completion` — submit `i3c_xfers_cb`, semaphore-wait on the callback, assert success.
- `test_rtio_iodev_submit_round_trip` — minimal RTIO sqe → cqe round trip via the fallback iodev.

**M4 commit**: "drivers: i3c: i3c_emul async callback + RTIO submit"

---

## Milestone M5 — Target mode + secondary controller handoff

### M5.1 — `target_register` / `target_unregister`
Store the `struct i3c_target_config *` in `i3c_emul_data` (mirror `i2c_emul_target_register`, `drivers/i2c/i2c_emul.c:271-291`). `i3c_xfers` checks for active target config and forwards to `cfg->callbacks->{write_received, read_requested, …}` from `include/zephyr/drivers/i3c/target_device.h` (`struct i3c_target_callbacks`, lines ~120-200).

### M5.2 — `target_tx_write`
Buffer the bytes in a per-target FIFO; the next read-direction `i3c_xfers` consumes them. Honor `hdr_mode` argument (just store it for the test to inspect).

### M5.3 — `target_controller_handoff`
Toggle `is_secondary` in `struct i3c_config_controller`. Notify any registered handoff callback (none required to actually move state; the emulator just records the request).

### M5.4 — Tests
**File**: `tests/drivers/i3c/i3c_emul/src/test_target.c` (new)
- `test_target_register_then_xfer_invokes_callbacks` — register target callbacks, send `i3c_xfers` with that addr, assert read_requested/write_received fire.
- `test_target_tx_write_fifo` — call `target_tx_write` with bytes, perform `i3c_read` from the controller side, assert bytes match.
- `test_controller_handoff_records` — call `target_controller_handoff(true)`, assert `is_secondary == true`.

**M5 commit**: "drivers: i3c: i3c_emul target-mode and controller-handoff hooks"

---

## Milestone M6 — Reference peripheral emulator

**Goal**: a single, well-documented peripheral emulator that exercises the full `struct i3c_emul_api` so subsequent peripheral emul authors have a template, and so the M7 tests have a real device to talk to.

### M6.1 — DT binding for the test target
**File**: `tests/drivers/i3c/i3c_emul/dts/bindings/zephyr,i3c-test-target.yaml` (or under `dts/bindings/test/` if it should be repo-wide)
- `compatible: "zephyr,i3c-test-target"`
- `include: i3c-device.yaml`
- additional optional props: `pid`, `bcr`, `dcr`, `ibi-capable`.

### M6.2 — Implement test target emul
**File**: `tests/drivers/i3c/i3c_emul/src/test_target_emul.c` (new)

A minimal device with:
- 32-byte register file (so `xfers` reads/writes are deterministic).
- Responds to `GETPID`/`GETBCR`/`GETDCR`/`GETSTATUS`/`GETMXDS`/`SETMWL`/`SETMRL` CCCs with values from its config.
- Implements `set_dynamic_addr` callback so M2.7's DAA flow assigns it cleanly.
- Implements `ibi_enable`/`ibi_disable` plus a `backend_api` function `test_target_trigger_ibi(emul, payload, len)` that calls `i3c_emul_target_raise_ibi`.
- Uses `EMUL_DT_INST_DEFINE` with both `bus_api` (the `i3c_emul_api`) and `backend_api` (the test-side trigger API).

### M6.3 — Build hookup
- `Kconfig` for the test target under the test directory (`config EMUL_ZEPHYR_I3C_TEST_TARGET`).
- `CMakeLists.txt` adds the source.

**M6 commit**: "tests: drivers: i3c: add reference i3c test-target peripheral emul"

---

## Milestone M7 — Test suite

**File**: `tests/drivers/i3c/i3c_emul/CMakeLists.txt`, `prj.conf`, `boards/native_sim.overlay`, `src/main.c`, plus the per-feature test files written in M2/M3/M4/M5.

### M7.1 — Devicetree overlay
```dts
&i3c0 {  /* declared in native_sim, or a fresh node */
    compatible = "zephyr,i3c-emul-controller";
    status = "okay";
    #address-cells = <3>;
    #size-cells = <0>;

    test_target_a: target@5500001234012345678 {
        compatible = "zephyr,i3c-test-target";
        reg = <0x55 0x12340 0x12345678>;  /* static addr + PID halves */
    };
    test_target_b: target@0000005678ABCDEF012 {
        compatible = "zephyr,i3c-test-target";
        reg = <0x00 0x5678A 0xBCDEF012>;  /* DAA-only */
        assigned-address = <0x42>;
    };
};
```

### M7.2 — `prj.conf`
```
CONFIG_ZTEST=y
CONFIG_I3C=y
CONFIG_I3C_CONTROLLER=y
CONFIG_I3C_TARGET=y
CONFIG_I3C_USE_IBI=y
CONFIG_I3C_CALLBACK=y
CONFIG_I3C_RTIO=y
CONFIG_I3C_EMUL=y
CONFIG_EMUL=y
CONFIG_EMUL_ZEPHYR_I3C_TEST_TARGET=y
CONFIG_LOG=y
```

### M7.3 — `src/main.c`
Standard ztest entry: `ZTEST_SUITE(i3c_emul, NULL, NULL, NULL, NULL, NULL);`. The per-feature `test_*.c` files (from M2/M3/M4/M5) all hang off this suite.

### M7.4 — Twister metadata
**File**: `tests/drivers/i3c/i3c_emul/testcase.yaml`
```yaml
tests:
  drivers.i3c.emul:
    platform_allow:
      - native_sim
    tags: drivers i3c emul
```

### M7.5 — Build-all coverage
**File**: `tests/drivers/build_all/i3c/` already exists; add `CONFIG_I3C_EMUL=y` to one of its variants (or add a new variant) so the emulator is included in the i3c build-all matrix going forward.

**M7 commit**: "tests: drivers: i3c: i3c_emul controller test suite"

---

## Verification (end-to-end)

After all milestones, from `/Users/ryanmcclelland/zephyrproject/zephyr`:

1. **Header sanity** — clean build, no warnings:
   ```bash
   west build -p auto -b native_sim samples/hello_world
   ```
   confirms the `emul.h` changes don't break unrelated builds.

2. **Twister run** — primary gate:
   ```bash
   ./scripts/twister -T tests/drivers/i3c/i3c_emul -p native_sim -v
   ```
   Expected: every `test_*` from M2/M3/M4/M5 passes.

3. **Build-all i3c** — confirms binding/Kconfig don't break the matrix:
   ```bash
   ./scripts/twister -T tests/drivers/build_all/i3c -v
   ```

4. **Lint**:
   ```bash
   ./scripts/checkpatch.pl --no-tree -f drivers/i3c/i3c_emul.c \
                                       include/zephyr/drivers/i3c_emul.h \
                                       tests/drivers/i3c/i3c_emul/src/*.c
   ```

5. **Smoke a real consumer** — pick one i3c-using sample (e.g. `samples/sensor/lps22hh_i3c`), build it for `native_sim` after rewriting its DT to use `zephyr,i3c-emul-controller`. The driver should `attach`, `do_daa`, and read a sample without ENOTSUP. (Optional but high-signal.)

6. **Coverage spot check** — run twister with `--coverage` against the i3c_emul test suite; confirm `drivers/i3c/i3c_emul.c` line coverage is meaningfully high (>70% for first cut).

---

## Out of scope / explicit non-goals

- **HDR-TSP / HDR-TSL / HDR-BT** modes. Only HDR-DDR is wired through (peripheral may opt in via the `hdr_ddr_xfers` callback). The other HDR variants are listed in `i3c_msg.flags` but no real driver in tree exercises them today.
- **Bus topology**: single-controller, single-bus only. No multi-controller bus arbitration simulation.
- **Timing accuracy**: SCL frequencies and OD timings are stored but not enforced.
- **CRC / parity** generation for HDR-DDR — peripheral side doesn't need to compute parity (test code can fabricate).
- **The `forwards` analogue** of `i2c_emul.c`. Skip in v1; can be added later if a use case appears.
