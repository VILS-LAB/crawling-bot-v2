# 3D-printable parts

Printable body parts for the crawling robot.

> **Placeholder.** No models are committed yet. Add them here and then fill in
> the tables below; [../HARDWARE.md](../../HARDWARE.md) links to this page.

## Layout

```text
stl/        printable meshes, pre-oriented, millimetres
step/       editable CAD (STEP AP214) with the same part names
drawings/   dimensioned 2D PDFs for parts that are not printed
```

## Print settings

> **Placeholder.** Fill this table in when the first models land. It belongs
> here, next to the files, so a reprint reproduces the original part.

| Setting | Value | Why |
|---|---|---|
| Material | _TBD_ | |
| Layer height | _TBD_ mm | |
| Walls | _TBD_ | |
| Infill | _TBD_ % | |
| Supports | _TBD_ | |

## Parts list

> **Placeholder.** One row per printed part.

| Part | File | Quantity | Notes |
|---|---|---|---|
| _TBD_ | `stl/_` | | |

## Assembly notes

* Set both legs to the **park pose (180/180)** before attaching the horns; the
  firmware parks there at power-on. See [../../CALIBRATION.md](../../CALIBRATION.md).
* Post-process holes that take bearings or servo horns — a tight fit will crack
  the part when a servo stalls.
