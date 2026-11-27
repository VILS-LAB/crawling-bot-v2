# Hardware files

Mechanical and electrical source files for the robot. This directory is the
**single home** for anything that is not firmware source code.

> ⚠️ **These files are being added.** Every subdirectory exists with a README
> that states what will land there, the naming rules, and what documentation it
> will feed. Add files following those rules so the rest of the docs keep
> working; then the corresponding pages can be completed.

## Layout

| Path | Contents | Status |
|---|---|---|
| [`3d/stl/`](3d/README.md) | printable models, pre-oriented | empty |
| [`3d/step/`](3d/README.md) | editable CAD exports | empty |
| [`3d/drawings/`](3d/README.md) | dimensioned 2D drawings (PDF) | empty |
| [`pcb/`](pcb/README.md) | KiCad project and fabrication output | empty |
| [`bom/`](bom/README.md) | component spreadsheet | empty |
| [`img/`](img/README.md) | photographs used by the documentation | empty |

## Rules

1. **One file, one home.** A part number lives in the BOM; the same number does
   not get retyped into a Markdown table that can drift.
2. **Reference, don't duplicate.** Documents link to the files here rather than
   restating dimensions or prices.
3. **Name for the human, not the tool.** `leg-upper-left.stl`, not
   `Untitled(3).stl`. Prefix revisions with a date: `2026-11-27-leg-upper.stl`.
4. **Ship the source format too.** STL for printing *and* STEP for editing; PCB
   project *and* gerbers.
5. **Units and scale.** Millimetres everywhere. STL in mm, STEP in mm, drawings
   dimensioned in mm.

## Documentation that depends on these files

Once files exist, these pages need their placeholders replaced with real content:

| Page | Needs |
|---|---|
| [../HARDWARE.md](../HARDWARE.md) | BOM table, print settings pointer, PCB summary |
| [../BUILD.md](../BUILD.md) | assembly order cross-links |
| [../PROJECT.md](../PROJECT.md) | per-phase hardware prerequisites |
