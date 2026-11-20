#!/usr/bin/env python3
"""Parallel offline bench: closed-loop autonomous_trials runs against rover_sim.

Each case is a full, isolated stack -- rover_sim + autonomous_trials.launch.py
(start_rviz:=false) from the bench install -- on its own ROS_DOMAIN_ID (Fast DDS,
localhost only), so up to --parallel cases run at once and never see each other
or the live system (rmw_zenoh).

    python3 scripts/bench/run_bench.py scripts/bench/cases/lawnmower_cusp.yaml --out /tmp/bench_x
    python3 scripts/bench/run_bench.py CASES.yaml --out DIR --only lane25_off lane25_cusp30
    python3 scripts/bench/run_bench.py --cleanup        # kill anything a dead runner left behind

Cases file (YAML):
    defaults:                       # merged into every case (args dicts are merged too)
      yaw_scale: 0.7                # rover_sim: actual yaw rate = commanded x this
      start: [-1.5, -1.0, 0.5]      # rover_sim start pose [x, y, yaw rad]
      args: {waypoint_mode: lawnmower, lawnmower_drive: zigzag_heading}
    cases:
      - name: lane25_cusp30
        start: [-1.71, 0.47, 1.548]
        resume_at: 25               # pre-create initial_bag + bag_01..24: only trial 25 runs
        n: 25                       # n_trajectories (default: resume_at, else 1)
        args: {lawnmower_cusp_tolerance: 30}
        timeout: 400                # s (default 120 + 150 per trial to run, + 150 for an initial bag)
        silence_timeout: 300        # s without an autonomous_trials log line = hung (default 300)

How a case ends (checked every 2 s; status in DIR/status.txt):
  done      autonomous_trials exited 0 (detected from the process, not from log text)
  crashed   autonomous_trials exited non-zero (e.g. a FATAL sys.exit or a Traceback)
  retry     Nav2 bringup aborted, or autonomous_trials still stuck in its Nav2-readiness
            wait 90 s after Nav2 came up -> restarted, up to 3 tries
  failed    a Nav2 node died / logged FATAL while the case ran, rover_sim died, the
            launch exited early, or bringup failed 3 times -- fails at once, no waiting
  hung      autonomous_trials silent for silence_timeout (its own 60 s stall check
            normally reports a stuck trial well before that)
  timeout   overall per-case limit
Nav2 bringups run one at a time (the next case starts once the previous one's
Nav2 is active, at most 60 s later) -- overlapping bringups made DDS service
replies (lifecycle get_state) time out; the driving runs in parallel.
Stopping a case signals every process it started, including ones that left its
process group (waypoint.launch.py, ros2 bag record). Every case process carries
the env var CUBEROVER_BENCH_RUNNER=<runner pid>: if the runner itself is killed,
--cleanup (also run automatically at every start) stops whatever it left behind. Domains with a
leftover ROS process on them are skipped. Warns if a package's source is newer
than the bench install (rebuild: scripts/bench/build_bench.sh).

At the end: DIR/summary.txt and, per case, DIR/<case>/tracks.png (plot_runs.py).
System python3 (no venv needed); psutil + yaml.
"""
import argparse
import ctypes
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

import psutil
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
ROVER_SIM = REPO / "scripts" / "mppi_offline_test" / "rover_sim.py"
DOMAINS = list(range(42, 62))
MARKER = "CUBEROVER_BENCH_RUNNER"   # env var on every process a case starts: the runner's pid
BRINGUP_FAIL = ("Aborting bringup", "Failed to bring up all requested nodes")
# top-level launch lines only (nested autonomous_trials/waypoint.launch lines are prefixed)
NODE_DIED = re.compile(r"^\[ERROR\] \[(?!autonomous_trials)([\w]+)-\d+\]: process has died \[pid \d+, exit code (-?\d+)")
NODE_FATAL = re.compile(r"^\[(\w+)-\d+\] \[FATAL\]")
ORCH_ERROR = re.compile(r"^\[autonomous_trials-\d+\] (FATAL: .*|\w+(Error|Exception)\b.*)")
ORCH_EXIT = re.compile(r"^\[(INFO|ERROR)\] \[autonomous_trials-\d+\]: process has (finished cleanly|died \[pid \d+, exit code (-?\d+))")
WATCHED_PKGS = ("nav2_stack", "nav2_smac_planner_custom", "nav2_mppi_controller")


def _die_with_parent():
    """preexec_fn: new session + SIGTERM when the runner dies (Linux prctl PR_SET_PDEATHSIG)."""
    os.setsid()
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)
    except OSError:
        pass


def ros_env_cmd(install_bench: Path, cmd: str) -> list:
    src = (f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && "
           f"source {install_bench}/local_setup.bash && exec {cmd}")
    return ["bash", "-c", src]


def merge(defaults: dict, case: dict) -> dict:
    out = {**defaults, **case}
    out["args"] = {**defaults.get("args", {}), **case.get("args", {})}
    return out


def kill_tree(pids, grace=15.0):
    """SIGINT every process in the given trees (snapshotted first, so children that
    left the process group are included), then SIGKILL whatever is left."""
    procs = []
    for pid in pids:
        try:
            p = psutil.Process(pid)
            procs += [p] + p.children(recursive=True)
        except psutil.NoSuchProcess:
            pass
    for p in procs:
        try:
            p.send_signal(signal.SIGINT)
        except psutil.Error:
            pass
    _, alive = psutil.wait_procs(procs, timeout=grace)
    for p in alive:
        try:
            p.kill()
        except psutil.Error:
            pass
    psutil.wait_procs(alive, timeout=3)


def _runner_alive(pid: int) -> bool:
    try:
        return "run_bench.py" in " ".join(psutil.Process(pid).cmdline())
    except psutil.Error:
        return False


def cleanup_stale(verbose=True) -> int:
    """Kill every process of this user that carries the MARKER env var of a runner that
    is no longer alive. Children inherit the variable, so this finds them however they
    were re-parented (a SIGKILLed runner leaves the Nav2 nodes behind)."""
    stale = []
    for p in psutil.process_iter(["uids"]):
        try:
            if p.info["uids"].real != os.getuid() or p.pid == os.getpid():
                continue
            owner = p.environ().get(MARKER)
            if owner and owner.isdigit() and not _runner_alive(int(owner)):
                stale.append(p.pid)
        except psutil.Error:
            pass
    if stale:
        if verbose:
            print(f"cleanup: stopping {len(stale)} process(es) left by a dead bench runner")
        kill_tree(stale, grace=5.0)
    return len(stale)


def domains_in_use() -> set:
    """ROS domains a running process of this user was started on (leftovers, other benches)."""
    used = set()
    for p in psutil.process_iter(["uids", "cmdline"]):
        try:
            if p.info["uids"].real != os.getuid():
                continue
            cmd = " ".join(p.info["cmdline"] or [])
            if not any(s in cmd for s in ("ros2", "nav2", "rover_sim", "lib/nav2_stack")):
                continue
            d = p.environ().get("ROS_DOMAIN_ID")
            if d is not None and d.isdigit():
                used.add(int(d))
        except psutil.Error:
            pass
    return used


def stale_build_warnings(install_bench: Path) -> list:
    out = []
    for pkg in WATCHED_PKGS:
        # colcon rewrites this on every build (installed files keep the source mtime)
        stamp = install_bench.parent / install_bench.name.replace("install", "build", 1) / pkg / "colcon_build.rc"
        src = REPO / "src" / pkg
        if not stamp.exists() or not src.exists():
            continue
        built = stamp.stat().st_mtime
        newer = [p for p in src.rglob("*") if p.is_file() and p.suffix in
                 (".py", ".cpp", ".hpp", ".yaml", ".xml", ".pgm", ".txt") and p.stat().st_mtime > built + 1]
        if newer:
            out.append(f"WARNING: {pkg}: {len(newer)} source file(s) newer than the bench install "
                       f"(e.g. {newer[0].relative_to(REPO)}) -- rebuild: scripts/bench/build_bench.sh {pkg}")
    return out


class Case:
    def __init__(self, spec: dict, out: Path, install_bench: Path):
        self.spec, self.name = spec, spec["name"]
        self.dir = out / self.name
        self.install_bench = install_bench
        self.resume_at = int(spec.get("resume_at", 0))
        self.n = int(spec.get("n", self.resume_at or 1))
        new_trials = self.n - max(self.resume_at - 1, 0)
        self.timeout = float(spec.get("timeout", 120 + 150 * new_trials + (0 if self.resume_at else 150)))
        self.silence_timeout = float(spec.get("silence_timeout", 300))
        self.procs, self.domain, self.t0, self.tries = [], None, None, 0
        self.state, self.detail = "queued", ""

    def start(self, domain: int):
        self.domain, self.tries, self.t0 = domain, self.tries + 1, time.monotonic()
        self.state, self.detail = "running", f"domain {domain}, try {self.tries}"
        self.dir.mkdir(parents=True, exist_ok=True)
        bags = self.dir / "bags"
        if bags.exists():
            subprocess.run(["rm", "-rf", str(bags)], check=True)
        bags.mkdir()
        if self.resume_at:
            (bags / "initial_bag").mkdir()
            for k in range(1, self.resume_at):
                (bags / f"bag_{k:02d}").mkdir()
        env = {**os.environ, "ROS_DOMAIN_ID": str(domain), "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
               "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
               "PYTHONUNBUFFERED": "1", MARKER: str(os.getpid())}
        start = self.spec.get("start", [-1.5, -1.0, 0.5])
        sim = (f"python3 {ROVER_SIM} --ros-args -p 'start:={list(map(float, start))}' "
               f"-p frame:=FitRosey_passive -p yaw_scale:={float(self.spec.get('yaw_scale', 1.0))}")
        largs = " ".join(shlex.quote(f"{k}:={v}") for k, v in self.spec["args"].items())
        trials = (f"ros2 launch nav2_stack autonomous_trials.launch.py bag_dir:={bags} "
                  f"n_trajectories:={self.n} start_rviz:=false {largs}")
        self.sim_log = open(self.dir / "sim.log", "w")
        self.log_path = self.dir / "trials.log"
        self.trials_log = open(self.log_path, "w")
        self.procs = [subprocess.Popen(ros_env_cmd(self.install_bench, sim), env=env, stdout=self.sim_log,
                                       stderr=subprocess.STDOUT, preexec_fn=_die_with_parent)]
        time.sleep(2.0)
        self.procs.append(subprocess.Popen(ros_env_cmd(self.install_bench, trials), env=env,
                                           stdout=self.trials_log, stderr=subprocess.STDOUT,
                                           preexec_fn=_die_with_parent))
        self.orchestrator_seen = False
        self.last_orch_line, self.last_orch_t, self.log_pos, self.log_tail = None, time.monotonic(), 0, ""
        self.exit_line, self.last_error, self.nav2_active = None, None, False
        self.nav2_active_t, self.first_goal = None, False

    def _orchestrator_alive(self) -> bool:
        try:
            kids = psutil.Process(self.procs[1].pid).children(recursive=True)
        except psutil.NoSuchProcess:
            return False
        for k in kids:
            try:
                if "lib/nav2_stack/autonomous_trials" in " ".join(k.cmdline()):
                    return True
            except psutil.Error:
                pass
        return False

    def _new_lines(self) -> list:
        with open(self.log_path, errors="replace") as f:
            f.seek(self.log_pos)
            chunk = f.read()
            self.log_pos = f.tell()
        text = self.log_tail + chunk
        lines = text.split("\n")
        self.log_tail = lines.pop()          # keep an unfinished last line for next time
        return lines

    def poll(self):
        """-> (verdict, reason); verdict in running|done|crashed|retry|failed|hung|timeout"""
        now = time.monotonic()
        for line in self._new_lines():
            if any(s in line for s in BRINGUP_FAIL):
                return "retry", "Nav2 bringup aborted"
            if "[autonomous_trials] " in line:
                self.last_orch_t, self.last_orch_line = now, line.split("[autonomous_trials] ", 1)[1][:120]
            m = ORCH_EXIT.match(line)
            if m:
                self.exit_line = m
            m = ORCH_ERROR.match(line)
            if m:
                self.last_error = m.group(1)[:160]
            if "[lifecycle_manager_navigation]" in line and "Managed nodes are active" in line:
                self.nav2_active, self.nav2_active_t = True, now
            if "[autonomous_trials] " in line and "waiting for goal" in line:
                self.first_goal = True
            m = NODE_DIED.match(line)
            if m and self.exit_line is None:
                return "failed", f"{m.group(1)} died (exit code {m.group(2)}) while the case ran"
            m = NODE_FATAL.match(line)
            if m and self.exit_line is None and m.group(1) != "autonomous_trials":
                return "failed", f"{m.group(1)} logged FATAL: {line.split('FATAL]', 1)[1][:140].strip()}"
        if self._orchestrator_alive():
            self.orchestrator_seen = True
        elif self.orchestrator_seen or self.exit_line is not None:
            self._new_lines_flush()
            m = self.exit_line
            if m is None or m.group(2) == "finished cleanly":
                return "done", ""
            return "crashed", f"autonomous_trials exit code {m.group(3)}: {self.last_error or self.last_orch_line or ''}"
        elif self.procs[1].poll() is not None:
            return "failed", "the launch exited before autonomous_trials started"
        if self.procs[0].poll() is not None:
            return "failed", "rover_sim died (see sim.log)"
        if now - self.t0 > self.timeout:
            return "timeout", f"timed out after {self.timeout:.0f}s"
        if (not self.first_goal and self.nav2_active_t is not None and now - self.nav2_active_t > 90
                and now - self.last_orch_t > 90):
            # orchestrator still in its Nav2-readiness wait long after Nav2 came up: the
            # lifecycle get_state exchange is wedged (seen on the bench) -- a bringup-class
            # failure, so restart the case
            return "retry", f"autonomous_trials stuck at start-up: {self.last_orch_line}"
        if self.orchestrator_seen and now - self.last_orch_t > self.silence_timeout:
            return "hung", (f"autonomous_trials silent for {self.silence_timeout:.0f}s; last line: "
                            f"{self.last_orch_line}")
        self.detail = (f"domain {self.domain}, try {self.tries}, {now - self.t0:.0f}s"
                       + (f" | {self.last_orch_line}" if self.last_orch_line else ""))
        return "running", ""

    def bringing_up(self) -> bool:
        """Still in Nav2 bringup (gates the next case's start; at most 60 s)."""
        return self.state == "running" and not self.nav2_active and time.monotonic() - self.t0 < 60

    def _new_lines_flush(self):
        """Give the launch a moment to log the orchestrator's exit status line."""
        for _ in range(10):
            if self.exit_line is not None:
                return
            time.sleep(0.3)
            for line in self._new_lines():
                m = ORCH_EXIT.match(line)
                if m:
                    self.exit_line = m

    def stop(self):
        kill_tree([p.pid for p in self.procs])
        self.procs = []
        # sweep: anything of this runner on this case's domain that appeared after the
        # snapshot above (e.g. a path_follower started while the orchestrator shut down)
        late = []
        for p in psutil.process_iter(["uids"]):
            try:
                if p.info["uids"].real != os.getuid():
                    continue
                e = p.environ()
                if e.get(MARKER) == str(os.getpid()) and e.get("ROS_DOMAIN_ID") == str(self.domain):
                    late.append(p.pid)
            except psutil.Error:
                pass
        if late:
            kill_tree(late, grace=5.0)
        for f in (getattr(self, "sim_log", None), getattr(self, "trials_log", None)):
            if f and not f.closed:
                f.close()

    def outcome_lines(self) -> list:
        out = []
        for line in self.log_path.read_text(errors="replace").splitlines():
            if "[autonomous_trials] " in line and any(s in line for s in (
                    "): waypoint reached", "): FAILED", "STALLED", "trials complete", "FATAL",
                    "_tolerance_deg=")):
                out.append(line.split("[autonomous_trials] ", 1)[-1])
            elif "[path_follower]: start pose reached" in line:
                out.append(line.split("] ", 2)[-1])
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="?")
    ap.add_argument("--out")
    ap.add_argument("--parallel", type=int, default=6)
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--install-bench", default=str(REPO / "install_bench"))
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--cleanup", action="store_true", help="only stop leftovers of dead runners, then exit")
    a = ap.parse_args()

    cleanup_stale()
    # Fast DDS shared-memory segments left by killed processes ("open_and_lock_file
    # failed" on start-up); only removes ones no running process holds
    subprocess.run(["bash", "-c", "source /opt/ros/jazzy/setup.bash && fastdds shm clean"],
                   capture_output=True)
    if a.cleanup:
        return
    if not a.cases or not a.out:
        ap.error("cases and --out are required")
    cfg = yaml.safe_load(Path(a.cases).read_text())
    specs = [merge(cfg.get("defaults", {}), c) for c in cfg["cases"]]
    if a.only:
        specs = [s for s in specs if s["name"] in a.only]
    out = Path(a.out).resolve(); out.mkdir(parents=True, exist_ok=True)
    install_bench = Path(a.install_bench).resolve()
    if not (install_bench / "local_setup.bash").exists():
        sys.exit(f"no bench install at {install_bench} -- run scripts/bench/build_bench.sh first")
    warnings = stale_build_warnings(install_bench)
    for w in warnings:
        print(w)
    busy = domains_in_use()
    free = [d for d in DOMAINS if d not in busy]
    if busy & set(DOMAINS):
        print(f"skipping ROS domains already in use: {sorted(busy & set(DOMAINS))}")
    if len(free) < min(a.parallel, len(specs)):
        sys.exit(f"only {len(free)} free bench domains")
    cases = [Case(s, out, install_bench) for s in specs]
    # longest first: bringups are serialized, so a long case started last sets the wall time
    queue, running = sorted(cases, key=lambda c: -c.timeout), []
    t_start = time.monotonic()
    def write_status():
        rows = [f"{c.name:24s} {c.state:9s} {c.detail}" for c in cases]
        (out / "status.txt").write_text(f"{time.monotonic() - t_start:.0f}s elapsed, {len(running)} running, "
                                        f"{len(queue)} queued\n" + "".join(w + "\n" for w in warnings)
                                        + "\n".join(rows) + "\n")

    def shutdown(*_):
        for c in running:
            c.stop()
        sys.exit("interrupted -- stopped all bench cases")
    signal.signal(signal.SIGINT, shutdown); signal.signal(signal.SIGTERM, shutdown)

    while queue or running:
        # one Nav2 bringup at a time: overlapping bringups make DDS service replies
        # (lifecycle get_state) time out; the driving itself runs in parallel
        if queue and len(running) < a.parallel and not any(c.bringing_up() for c in running):
            c = queue.pop(0); c.start(free.pop(0)); running.append(c)
        time.sleep(2.0)
        for c in list(running):
            verdict, reason = c.poll()
            if verdict == "running":
                continue
            c.stop(); running.remove(c); free.append(c.domain)
            if verdict == "retry" and c.tries < 3:
                c.state, c.detail = "queued", f"{reason} (try {c.tries}), retrying"
                queue.insert(0, c)
            else:
                c.state = "failed" if verdict == "retry" else verdict
                c.detail = f"{time.monotonic() - c.t0:.0f}s" + (f": {reason}" if reason else "")
                if verdict == "retry":
                    c.detail += " -- 3 tries"
        write_status()

    summary = list(warnings)
    for c in cases:
        summary.append(f"=== {c.name}: {c.state} ({c.detail})")
        summary += ["  " + s for s in c.outcome_lines()]
        if not a.no_plots and (c.dir / "bags").exists():
            r = subprocess.run(ros_env_cmd(install_bench, f"python3 {HERE / 'plot_runs.py'} {c.dir / 'bags'} "
                                                          f"{c.dir / 'tracks.png'} --title {shlex.quote(c.name)}"),
                               capture_output=True, text=True)
            summary += ["  " + s for s in r.stdout.splitlines()]
            if r.returncode:
                err = r.stderr.strip().splitlines()
                summary.append(f"  (plot failed: {err[-1] if err else r.returncode})")
    (out / "summary.txt").write_text("\n".join(summary) + "\n")
    print("\n".join(summary))
    bad = [c.name for c in cases if c.state != "done"]
    print(f"\n{len(cases)} case(s) in {time.monotonic() - t_start:.0f}s -> {out}"
          + (f"; NOT done: {', '.join(bad)}" if bad else ""))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
