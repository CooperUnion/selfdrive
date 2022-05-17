class PIDController():
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, Ts=0.005, llim=-100.0, ulim=100.0, sigma=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.Ts = Ts
        self.llim = llim
        self.ulim = ulim
        self.sigma = sigma
        self.beta = (2.0*self.sigma - self.Ts) / (2.0*self.sigma + self.Ts)
        self.y_d1 = 0
        self.error_d1 = 0
        self.y_dot = 0
        self.error_dot = 0
        self.integrator = 0

    def PID(_, y_r, y):
        # Compute current error
        error = y_r - y
        # Integrate erro rusing trapazoidal rule
        _.integrator = _.integrator + ((_.Ts/2) * (error + _.error_d1))

        # Prevent integrator unsaturation
        if _.ki != 0.0:
            integrator_unsat = _.ki*_.integrator
            _.integrator=_.saturate(integrator_unsat)/_.ki
        
        # Differentiate error
        _.error_dot = _.beta*_.error_dot + (((1-_.beta)/_.Ts) * (error - _.error_d1))

        # PID Control
        u_unsat = (_.kp * error) + (_.ki * _.integrator) + (_.kd * _.error_dot)

        # Saturate control input
        u_sat = _.saturate(u_unsat)

        _.error_d1 = error
        _.y_d1 = y
        return u_sat

    def saturate(self, u):
        return max(min(self.ulim, u), self.llim)
    
    def update_time_parameters(self, Ts, sigma):
        self.Ts = Ts
        self.sigma = sigma
        self.beta = (2.0*self.sigma - self.Ts) / (2.0*self.sigma + self.Ts)
    
    def setpoint_reset(self, y_r, y):
        self.integrator = 0
        self.error_d1 = y_r - y
        self.error_dot = 0
    
    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd