"""Robot visualizer: live plots, instruments and a protocol test console.

Layout of the package:

    telemetry.py  data format, parser, ring buffer   (no GUI)
    panels.py     what is plotted and where          (no GUI)
    health.py     boot log + telemetry -> checklist  (no GUI)
    console.py    command buttons and reply state    (no GUI)
    drawing.py    blitter and scene geometry         (matplotlib, no widgets)
    sources.py    serial / tcp / demo / replay       (threads)
    app.py        the window

The first five have no widget dependency, so they are importable and testable
headlessly; see tools/test_robot_viewer.py.
"""

__version__ = "1.0.3"
