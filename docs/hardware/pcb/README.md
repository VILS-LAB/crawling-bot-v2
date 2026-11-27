# PCB

> **Placeholder.** The board design is not committed yet. When it lands, keep the
> project file and the fabrication output together in this directory.

## Layout

```text
*.kicad_pro / *.kicad_sch / *.kicad_pcb   design source (KiCad)
gerbers/                                  fabricator output, one zip per revision
bom/                                      assembly BOM in the fabricator's format
```

## What to document here

> **Placeholder.** Fill in when the first revision exists.

| Item | Value |
|---|---|
| Revision | _TBD_ |
| Layers / thickness | _TBD_ |
| Connectors | _TBD_ (servo headers, I2C header, USB, power) |
| Test points | _TBD_ |
| Known errata | _TBD_ |

## Relationship to the firmware pin map

The board's silkscreen and the firmware's [include/config.h](../../../include/config.h)
must agree. If a revision changes a servo pin, the header changes in the same
commit and [../../HARDWARE.md](../../HARDWARE.md) is updated — otherwise the boot
health check reports `ATTACH_FAILED`.
