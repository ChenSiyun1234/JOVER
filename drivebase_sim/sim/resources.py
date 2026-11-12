"""Resource and local-time guard for every simulation batch."""

from datetime import datetime
from zoneinfo import ZoneInfo
import time
import psutil


def wait_for_simulation():
    while True:
        now = datetime.now(ZoneInfo("America/Los_Angeles"))
        blocked = now.hour == 22 and 25 <= now.minute < 45
        free = psutil.virtual_memory().available / 2**30
        if free >= 1.5 and not blocked:
            print(f"Simulation guard: {free:.2f} GiB available, local {now.isoformat()}", flush=True)
            return
        print(f"Waiting: available={free:.2f} GiB, quiet_window={blocked}", flush=True)
        time.sleep(30)
