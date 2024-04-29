import time


class Timer:
    def __init__(self):
        self.starts = {}
        self.tots = {}
        self.logs = {}
        self.last_end = 0

    def duration_gt(self, thing, timeout, alt_now=None):
        return self.get_duration(thing, alt_now) > timeout

    def get_duration(self, thing, alt_now=None):
        if thing not in self.starts:
            raise Exception(f"{thing} has not been started.")
        else:
            return self.now(alt_now) - self.starts[thing]

    def now(self, alt_now=None):
        if alt_now is not None:
            return alt_now
        return time.clock_gettime(time.CLOCK_THREAD_CPUTIME_ID)

    def end_all(self, alt_now=None):
        for thing in self.starts:
            self.end(thing, alt_now)

    def start(self, thing, alt_now=None):
        if thing not in self.starts:
            now = self.now(alt_now)
            self.starts[thing] = now
        else:
            raise Exception(thing + " already started!")

    def restart(self, thing, alt_now=None):
        if thing not in self.starts:
            raise Exception(f"{thing} has not been started.")
        now = self.now(alt_now)
        self.end(thing, now)
        self.start(thing, now)

    def end(self, thing, alt_now=None):
        now = self.now(alt_now)
        self.last_end = now
        if thing not in self.starts:
            raise Exception(f"{thing} not in starts")
        run_time = now - self.starts[thing]
        if thing not in self.tots:
            self.tots[thing] = run_time
        else:
            self.tots[thing] += run_time
        del self.starts[thing]

    def end_from_last_end(self, thing, alt_now=None):
        self.starts[thing] = self.last_end
        self.end(thing, alt_now)

    def log(self, info, thing, alt_now=None):
        if thing not in self.logs:
            self.logs[thing] = {}
        self.logs[thing][self.get_duration(thing, alt_now)] = info

    def __str__(self):
        string = ''
        for thing in self.tots:
            string += thing + ' ' + str(self.tots[thing]) + '\n'
        return string

