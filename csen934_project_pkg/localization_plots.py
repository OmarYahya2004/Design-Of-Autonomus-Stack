"""
Open three rqt_plot windows (x, y, yaw) comparing GT vs Noisy vs Filtered.

rqt_plot silently drops any topic it cannot resolve when it starts and never
retries, so launching it together with Webots gives empty plots. This waits
until every plotted topic is being published, then starts one rqt_plot (with
topic retry, see rqt_plot_retry.py) per quantity and stays alive until they
are closed.
"""
import signal
import subprocess
import sys
import time

import rclpy

SOURCES = ['Filtered', 'Noisy', 'GT']   # GT added last so it is drawn on top
PLOTS = {
    'x': ['/Car_Odom_%s/pose/pose/position/x' % s for s in SOURCES],
    'y': ['/Car_Odom_%s/pose/pose/position/y' % s for s in SOURCES],
    'yaw': ['/Car_Yaw_%s/data' % s for s in SOURCES],
}
REQUIRED_TOPICS = ['/Car_Odom_%s' % s for s in SOURCES] + ['/Car_Yaw_%s' % s for s in SOURCES]
POLL_PERIOD = 0.5     # s
REPORT_PERIOD = 10.0  # s


def wait_for_topics(node):
    last_report = time.monotonic()
    while rclpy.ok():
        published = {
            name for name, _ in node.get_topic_names_and_types()
            if node.count_publishers(name) > 0
        }
        missing = [t for t in REQUIRED_TOPICS if t not in published]
        if not missing:
            return True
        if time.monotonic() - last_report > REPORT_PERIOD:
            node.get_logger().info('Waiting for topics: %s' % ', '.join(missing))
            last_report = time.monotonic()
        rclpy.spin_once(node, timeout_sec=POLL_PERIOD)
    return False


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('Localization_Plots_Node')
    node.get_logger().info('Waiting for odometry topics before opening rqt_plot...')
    try:
        ready = wait_for_topics(node)
    except KeyboardInterrupt:
        ready = False
    if ready:
        node.get_logger().info('Topics found, opening x / y / yaw plots')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not ready:
        return

    # Ctrl+C (SIGINT) or launch shutdown (SIGTERM) -> close the plot windows too.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    # rqt_plot with topic retry (see rqt_plot_retry.py). Started directly rather
    # than via `ros2 run`, which adds a wrapper process that would leave the real
    # window orphaned when stopped.
    plots = [
        subprocess.Popen([sys.executable, '-m', 'csen934_project_pkg.rqt_plot_retry', *topics])
        for topics in PLOTS.values()
    ]
    try:
        while any(p.poll() is None for p in plots):
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        close_plots(plots)


def close_plots(plots, grace=3.0):
    # rqt_plot does not always exit on SIGINT, so escalate to SIGKILL.
    for p in plots:
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
    deadline = time.monotonic() + grace
    for p in plots:
        try:
            p.wait(timeout=max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


if __name__ == '__main__':
    main()
