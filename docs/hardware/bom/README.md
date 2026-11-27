# Bill of materials

> **Placeholder.** The component spreadsheet is not committed yet.

## Expected file

`crawling-bot-v2-bom.xlsx` — one row per component, with the columns below. It
becomes the single source for ordering and for the assembly guide; documents
link here instead of repeating prices or part numbers.

| Column | Meaning |
|---|---|
| Ref | schematic reference (`U1`, `S1`, `M1`) |
| Part | manufacturer part number |
| Description | short human description |
| Qty | per robot |
| Package | footprint / mechanical size |
| Supplier | distributor |
| Unit price | in the currency used for ordering |
| Notes | substitutions, sourcing caveats |

## Also include

* a **mechanical** section (screws, bearings, horns, heat-set inserts),
* a **consumables** section (wire, connectors, tape),
* the **per-robot total**, and the count of robots the course needs.
