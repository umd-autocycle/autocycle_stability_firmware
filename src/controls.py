class Control:
    def get_control(self, goals):
        def no_control(t, e, v):
            return 0

        return no_control

    def reset_registers(self):
        pass


class PIDPhiInterpolated(Control):
    integral = 0
    last_time = 0
    first = 4
    final = 15
    interval = 0.5

    def __init__(self, max_torque):
        self.max_torque = max_torque
        self.params = [[33.69352243, 0.263932, 21.15690148],
                       [101.20843419, 4.4401922, 38.1772637],
                       [102.20843419, 5.4401922, 37.55922972],
                       [103.20843419, 5.8221582, 35.94119572],
                       [102.97236622, 5.96805622, 34.32316172],
                       [103.97236622, 6.35002222, 32.70512772],
                       [104.97236622, 6.49592025, 32.08709375],
                       [104.35433225, 6.18809353, 27.85102572],
                       [105.35433225, 7.18809353, 27.23299175],
                       [105.25222667, 7.33399156, 26.61495777],
                       [105.63419267, 7.21653726, 25.9969238],
                       [104.61175785, 7.09906907, 25.37888982],
                       [105.35554416, 7.62693309, 19.40675385],
                       [103.73751016, 8.00889909, 17.78871985],
                       [104.73751016, 8.39086509, 17.17068588],
                       [104.47474016, 8.77283109, 16.5526519],
                       [105.47474016, 8.15479712, 15.93461793],
                       [102.25887179, 8.53676312, 14.31658393],
                       [101.64083781, 8.91872912, 13.69854995],
                       [102.64083781, 9.30069512, 12.08051595],
                       [102.02280384, 8.68266114, 11.46248198],
                       [103.02280384, 9.68266114, 9.84444798],
                       [104.02280384, 10.68266114, 9.226414]]

    def get_control(self, goals):
        def pid_phi(t, e, v):
            self.integral += e[0] * (t - self.last_time)
            self.last_time = t

            index = int((v - self.first) / self.interval)
            interpolation = (v - self.first - index * self.interval) / self.interval

            k_p = self.params[index][0] + interpolation * (self.params[index + 1][0] - self.params[index][0])
            k_i = self.params[index][1] + interpolation * (self.params[index + 1][1] - self.params[index][1])
            k_d = self.params[index][2] + interpolation * (self.params[index + 1][2] - self.params[index][2])

            return min(self.max_torque, max(-self.max_torque, k_p * e[0] + k_d * e[2] + k_i * self.integral))

        return pid_phi

    def reset_registers(self):
        self.integral = 0
        self.last_time = 0
