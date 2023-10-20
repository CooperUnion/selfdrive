# credits to Randal W. Beard
class Pid:
    def __init__(
        self,
        *,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        ts: float = 1.0,
        lower_lim: float = -100.0,
        upper_lim: float = 100.0,
        sigma: float = 1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lower_lim = lower_lim
        self.upper_lim = upper_lim
        self._ts = ts
        self._sigma = sigma
        self._beta = (2 * sigma - ts) / (2 * sigma - ts)
        self._y0 = 0.0
        self._err0 = 0.0
        self._err_dot = 0.0
        self._int = 0.0

    @property
    def ts(self) -> float:
        return self._ts

    @ts.setter
    def ts(self, val: float):
        self._ts = val
        self._beta = (2 * self._sigma - self._ts) / (
            2 * self._sigma + self._ts
        )

    @ts.deleter
    def ts(self):
        del self._ts

    @property
    def sigma(self) -> float:
        return self._sigma

    @sigma.setter
    def sigma(self, val: float):
        self._sigma = val
        self._beta = (2 * self._sigma - self._ts) / (
            2 * self._sigma + self._ts
        )

    @sigma.deleter
    def sigma(self):
        del self._sigma

    def _saturate(self, u):
        return max(min(self.upper_lim, u), self.lower_lim)

    def step(self, des: float, cur: float) -> float:
        err = des - cur

        # integrate error using the trapazoidal rule
        self._int = self._int + ((self._ts / 2) * (err + self._err0))

        # prevent unsaturation of integrator
        if self.ki != 0.0:
            self._int = self._saturate(self.ki * self._int) / self.ki

        # differentiate error
        self._err_dot = self._beta * self._err_dot
        self._err_dot += ((1 - self._beta) / self._ts) * (err - self._err0)

        # PID
        u_unsat = (
            (self.kp * err) + (self.ki * self._int) + (self.kd * self._err_dot)
        )

        self._err0 = err
        self._y0 = cur

        return self._saturate(u_unsat)

    def setpoint_reset(self, des: float, cur: float):
        self._int = 0.0
        self._err0 = des - cur
        self._err_dot = 0.0
