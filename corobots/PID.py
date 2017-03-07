#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
####### Example #########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#


class PID:
    """
    Discrete PID control

    Args:
        P (float): proportional gain
        I (float): integral gain
        D (float): derivative gain
        Derivator (float): starting value of Derivator
        Intregrator (float): starting value of Integrator
        Integrator_max (float): maximum integrator value
        Integrator_min (float): minimum integrator value

    Attributes:
        Kp (float):proportional gain
        Ki (float): integral gain
        Kd (float): derivative gain
        Derivator (float): starting value of Derivator
        Intregrator (float): starting value of Integrator
        Integrator_max (float): maximum integrator value
        Integrator_min (float): minimum integrator value
        set_point (float): set point for controller
        error (float): error, set_point - current_value
        PID (float): output of controller
    """

    def __init__(self, P=1.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        """Calculate PID output value for given reference input and feedback.

        Args:
            current_value (float): current output of plant

        Returns:
            PID (float): controller output
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        self.PID = self.P_value + self.I_value + self.D_value

        return self.PID
        
    def update_sat(self, current_value):
        """Calculate PID output value for given reference input and feedback.
            Applies saturation at [-1, 1]

        Args:
            current_value (float): current output of plant

        Returns:
            PID (float): controller output
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        self.PID = self.P_value + self.I_value + self.D_value
        
        self.PID_sat = max(min(self.PID, 1), -1)
        
        return self.PID_sat

    def setPoint(self, set_point):
        """
        Initialize the setpoint of PID
        """
        self.set_point = set_point
        # self.Integrator = 0
        # self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator