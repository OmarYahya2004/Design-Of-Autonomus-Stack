"""
Stock rqt_plot with two fixes for plotting the localization topics.

1. Topic retry. Stock rqt_plot resolves the topics given on the command line
   exactly once, right after its node starts. At that moment the node has
   usually not finished discovering the existing topics (especially while
   Webots is rendering), so the topics are silently dropped and nothing is
   plotted. Here an unresolved topic is retried every second until it appears.

2. Y auto-scale. Stock rqt_plot only ever extends the y-axis, starting from
   [0, 1], so every plot keeps 0 in range (x is ~-45 m, yaw ~180 deg) and the
   noise is squashed into tiny spikes. Here the y-axis fits the data currently
   visible in the window, with a small margin.

3. Time window. When a line is added, matplotlib re-fits the time axis to the
   ~1 s of data received so far and autoscroll keeps that width. Here the
   window is reset to 30 s after each line is added.

Usage: python3 -m csen934_project_pkg.rqt_plot_retry /topic/field [...]
"""
import sys

from python_qt_binding.QtCore import QTimer
from rqt_plot import main as rqt_plot_main
from rqt_plot import plot_widget
from rqt_plot.data_plot import DataPlot

RETRY_MS = 1000
Y_MARGIN = 0.05        # fraction of the visible data range added above and below
TIME_WINDOW_S = 30.0   # seconds of data visible while autoscrolling

# --- 1. Topic retry --------------------------------------------------------
_original_add_topic = plot_widget.PlotWidget.add_topic


def _add_topic_with_retry(self, topic_name):
    fields, _ = plot_widget.get_plot_fields(self._node, topic_name)
    if not fields:
        QTimer.singleShot(RETRY_MS, lambda: _add_topic_with_retry(self, topic_name))
        return
    _original_add_topic(self, topic_name)
    # Adding a line makes matplotlib re-fit the time axis to the few samples
    # received so far (~1 s), and autoscroll then keeps that width. Restore it.
    if self.data_plot is not None:
        self.data_plot.set_xlim([0.0, TIME_WINDOW_S])


plot_widget.PlotWidget.add_topic = _add_topic_with_retry

# --- 2. Y auto-scale to the visible data -----------------------------------
_original_set_autoscale = DataPlot.set_autoscale
_original_set_ylim = DataPlot.set_ylim


def _set_autoscale_fit_visible(self, x=None, y=None):
    if y:
        y = y & ~DataPlot.SCALE_EXTEND   # fit visible data instead of only growing
    _original_set_autoscale(self, x=x, y=y)


def _set_ylim_with_margin(self, limits):
    low, high = float(limits[0]), float(limits[1])
    pad = (high - low) * Y_MARGIN
    _original_set_ylim(self, [low - pad, high + pad])


DataPlot.set_autoscale = _set_autoscale_fit_visible
DataPlot.set_ylim = _set_ylim_with_margin


def main():
    # --empty: don't restore topics saved by a previously closed rqt_plot. All
    # standalone rqt_plot windows share those saved settings, and because our
    # topics are added by retry (not at startup) rqt_plot would otherwise add
    # the last-closed window's topics as well, mixing x / y / yaw.
    sys.argv = ['rqt_plot', '--empty', *sys.argv[1:]]
    rqt_plot_main.main()


if __name__ == '__main__':
    main()
