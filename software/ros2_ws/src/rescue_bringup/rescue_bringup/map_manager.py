#!/usr/bin/env python3
"""
map_manager — always-on Jetson node that owns the robot's named map library, so
the workstation GUI can save / list / load / delete maps over ROS 2 (the map
files live on the robot; the GUI only mirrors names + preview thumbnails, exactly
like the arm's saved poses).

Layout — one directory per named map under `maps_dir` (default ~/maps):

    ~/maps/<name>/
        map.posegraph, map.data   # slam_toolbox serialized graph (2-D)
        occ.pgm, occ.yaml         # nav2 occupancy grid (2-D, for AMCL/map_server)
        map3d.bt                   # OctoMap binary tree (3-D)

Services (all under /robot/maps):

    /robot/maps/save    rescue_interfaces/SaveMap   name, kind("2d"|"3d")
    /robot/maps/list    rescue_interfaces/ListMaps  -> names + has_2d/has_3d
    /robot/maps/load    rescue_interfaces/LoadMap   name, kind("2d"|"3d"|"both")
    /robot/maps/delete  rescue_interfaces/DeleteMap name
    /robot/maps/status  std_msgs/String (latched)   "<n> map(s)"

How each kind is handled (robot-side, untestable off-hardware):
  • 2-D save  — `ros2 service call /slam_toolbox/serialize_map` (posegraph) +
                `ros2 run nav2_map_server map_saver_cli -t /map` (occupancy grid),
                the same calls as rescue_nav/save_competition_map.launch.py.
  • 3-D save  — forwarded to octomap_node's /robot/map3d/save (it owns the octree).
  • 2-D load  — write ~/.config/rescue/active_map.env (MAP_DIR/MAP_NAME) and
                `systemctl --user restart rescue-localization.service` (the unit
                launches AMCL + map_server on $MAP_DIR/occ.yaml). Then the operator
                sets the start pose by click-dragging on the map (GUI -> /initialpose).
  • 3-D load  — forwarded to octomap_node's /robot/map3d/load.

Subprocess (`ros2 service call`, `map_saver_cli`, `systemctl`) is used for the
external interactions on purpose — it avoids intra-node service-client reentrancy
on the single-threaded executor and mirrors the proven launch-file commands.

Parameters
  maps_dir          (str)   library root                 [~/maps]
  localization_unit (str)   2-D localization systemd unit [rescue-localization.service]
  env_file          (str)   active-map env for that unit  [~/.config/rescue/active_map.env]
  cmd_timeout       (float) per subprocess timeout, s     [30.0]
"""

import os
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from rescue_interfaces.srv import SaveMap, ListMaps, LoadMap, DeleteMap


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')

        home = os.path.expanduser('~')
        self.maps_dir = os.path.expanduser(
            self.declare_parameter('maps_dir', os.path.join(home, 'maps')).value)
        self.localization_unit = str(self.declare_parameter(
            'localization_unit', 'rescue-localization.service').value)
        self.env_file = os.path.expanduser(self.declare_parameter(
            'env_file', os.path.join(home, '.config', 'rescue', 'active_map.env')).value)
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 30.0).value)

        os.makedirs(self.maps_dir, exist_ok=True)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.status_pub = self.create_publisher(String, '/robot/maps/status', latched)

        self.create_service(SaveMap, '/robot/maps/save', self._on_save)
        self.create_service(ListMaps, '/robot/maps/list', self._on_list)
        self.create_service(LoadMap, '/robot/maps/load', self._on_load)
        self.create_service(DeleteMap, '/robot/maps/delete', self._on_delete)

        self._publish_status()
        self.get_logger().info(f'map_manager ready — maps_dir={self.maps_dir}')

    # ── helpers ────────────────────────────────────────────────────────────────
    def _map_path(self, name):
        return os.path.join(self.maps_dir, name)

    def _run(self, cmd):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=self.cmd_timeout)
            return r.returncode, (r.stdout + r.stderr).strip()
        except Exception as e:  # noqa: BLE001 - surface any failure to the caller
            return 1, str(e)

    def _service_call_ok(self, out):
        # `ros2 service call` prints the response; success services return
        # "success=True". Treat a missing field as failure.
        return 'success=True' in out.replace(' ', '')

    def _publish_status(self):
        try:
            n = len([d for d in os.listdir(self.maps_dir)
                     if os.path.isdir(self._map_path(d))])
        except OSError:
            n = 0
        self.status_pub.publish(String(data=f'{n} map(s)'))

    @staticmethod
    def _safe_name(name):
        name = (name or '').strip()
        # No path separators / traversal — names are single directory components.
        return name and ('/' not in name) and ('\\' not in name) and name not in ('.', '..')

    # ── save ────────────────────────────────────────────────────────────────────
    def _on_save(self, req, resp):
        if not self._safe_name(req.name):
            resp.success, resp.message = False, 'invalid map name'
            return resp
        d = self._map_path(req.name)
        os.makedirs(d, exist_ok=True)
        resp.path = d

        if req.kind == '2d':
            rc1, out1 = self._run([
                'ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                'slam_toolbox/srv/SerializePoseGraph',
                "{filename: '%s'}" % os.path.join(d, 'map')])
            rc2, out2 = self._run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', os.path.join(d, 'occ'), '-t', '/map',
                '--ros-args', '-p', 'save_map_timeout:=10.0'])
            ok = (rc2 == 0) and os.path.exists(os.path.join(d, 'occ.yaml'))
            resp.success = ok
            resp.message = ('saved 2-D map to %s' % d) if ok else \
                ('2-D save failed (slam_toolbox up + publishing /map?): %s | %s'
                 % (out1[-200:], out2[-200:]))
        elif req.kind == '3d':
            rc, out = self._run([
                'ros2', 'service', 'call', '/robot/map3d/save',
                'rescue_interfaces/srv/SaveMap',
                "{name: '%s', kind: '3d'}" % req.name])
            ok = (rc == 0) and self._service_call_ok(out)
            resp.success = ok
            resp.message = ('saved 3-D map to %s' % d) if ok else \
                ('3-D save failed (is rescue-mapping3d running?): %s' % out[-300:])
        else:
            resp.success, resp.message = False, f"unknown kind '{req.kind}' (want 2d|3d)"

        self._publish_status()
        (self.get_logger().info if resp.success else self.get_logger().error)(resp.message)
        return resp

    # ── list ────────────────────────────────────────────────────────────────────
    def _on_list(self, _req, resp):
        names, has_2d, has_3d = [], [], []
        try:
            entries = sorted(d for d in os.listdir(self.maps_dir)
                             if os.path.isdir(self._map_path(d)))
        except OSError:
            entries = []
        for name in entries:
            d = self._map_path(name)
            names.append(name)
            has_2d.append(os.path.exists(os.path.join(d, 'occ.yaml')))
            has_3d.append(os.path.exists(os.path.join(d, 'map3d.bt')))
        resp.names, resp.has_2d, resp.has_3d = names, has_2d, has_3d
        return resp

    # ── load ────────────────────────────────────────────────────────────────────
    def _on_load(self, req, resp):
        if not self._safe_name(req.name):
            resp.success, resp.message = False, 'invalid map name'
            return resp
        d = self._map_path(req.name)
        if not os.path.isdir(d):
            resp.success, resp.message = False, f"no map '{req.name}'"
            return resp

        msgs, ok_any = [], False
        if req.kind in ('2d', 'both'):
            if not os.path.exists(os.path.join(d, 'occ.yaml')):
                msgs.append('no 2-D occupancy grid for this map')
            else:
                os.makedirs(os.path.dirname(self.env_file), exist_ok=True)
                with open(self.env_file, 'w') as f:
                    f.write('MAP_DIR=%s\nMAP_NAME=%s\n' % (d, req.name))
                rc, out = self._run(
                    ['systemctl', '--user', 'restart', self.localization_unit])
                if rc == 0:
                    ok_any = True
                    msgs.append('2-D localization (re)started on %s; '
                                'set the start pose on the map' % req.name)
                else:
                    msgs.append('failed to start %s: %s' % (self.localization_unit, out[-200:]))

        if req.kind in ('3d', 'both'):
            rc, out = self._run([
                'ros2', 'service', 'call', '/robot/map3d/load',
                'rescue_interfaces/srv/LoadMap',
                "{name: '%s', kind: '3d'}" % req.name])
            if rc == 0 and self._service_call_ok(out):
                ok_any = True
                msgs.append('3-D octree loaded')
            else:
                msgs.append('3-D load failed (is rescue-mapping3d running?)')

        resp.success = ok_any
        resp.message = '; '.join(msgs) if msgs else f"unknown kind '{req.kind}'"
        (self.get_logger().info if resp.success else self.get_logger().error)(resp.message)
        return resp

    # ── delete ───────────────────────────────────────────────────────────────────
    def _on_delete(self, req, resp):
        if not self._safe_name(req.name):
            resp.success, resp.message = False, 'invalid map name'
            return resp
        d = self._map_path(req.name)
        if not os.path.isdir(d):
            resp.success, resp.message = False, f"no map '{req.name}'"
            return resp
        try:
            shutil.rmtree(d)
            resp.success, resp.message = True, f"deleted '{req.name}'"
        except OSError as e:
            resp.success, resp.message = False, f'delete failed: {e}'
        self._publish_status()
        return resp


def main():
    rclpy.init()
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
