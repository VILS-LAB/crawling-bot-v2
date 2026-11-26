"""Regression test: the viewer must size its grid from the data, not from
Q,CONFIG.

A viewer attached to a robot whose step was changed before the viewer started
used to draw a 25-row grid for a 100-state table, which made the Q table and
the greedy-policy panel look broken.

Run headless:
    MPLBACKEND=Agg python tools/test_viewer_geometry.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qlearn_visualizer as V


def table_lines(n_values, chunks):
    flat = [0.01 * i for i in range(n_values)]
    per = (n_values + chunks - 1) // chunks
    out = []
    for i in range(0, n_values, per):
        part = flat[i:i + per]
        out.append(f"Q,TABLE,{i // per + 1},{chunks},{len(part)},"
                   + ",".join(f"{v:.4f}" for v in part))
    return out


def check(label, lines, expect_shape):
    p = V.QParser()
    app = V.App(V.Config(), V.Source(V.queue.SimpleQueue(), p, V.Config()),
                V.queue.SimpleQueue(), None, "TEST")
    for ln in lines:
        ev = p.feed(ln)
        if ev:
            app.q.put(ev)
    app._drain()
    app._refresh()
    app.fig.canvas.draw()

    n = expect_shape[0] ** 0.5
    assert app.table.shape == expect_shape, f"{label}: table {app.table.shape} != {expect_shape}"
    assert app.im.get_array().shape == expect_shape, f"{label}: heatmap wrong"
    assert len(app.g_text) == int(n), f"{label}: policy grid rows {len(app.g_text)} != {int(n)}"
    assert len(app.g_text[0]) == int(n), f"{label}: policy grid cols wrong"
    print(f"  {label}: OK")


print("viewer geometry regression")
check("no CONFIG, 125 values (step 45)", table_lines(125, 2), (25, 5))
check("no CONFIG, 500 values (step 20)", table_lines(500, 5), (100, 5))
check("no CONFIG, 1805 values (step 10)", table_lines(1805, 19), (361, 5))
check("CONFIG then table",
      ["Q,CONFIG,step=20,notches=10,states=100,axis=Y,sign=-1,deadband=0.150,"
       "mode=2,pos=1.000,neg=-1.000,idle=-0.100,threshold=0.500,pause=400"]
      + table_lines(500, 5), (100, 5))
check("stale CONFIG, real 500 values",
      ["Q,CONFIG,step=45,notches=5,states=25"] + table_lines(500, 5), (100, 5))
print("all geometry checks passed")
