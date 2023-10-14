import trajectory as t
import unittest

class BasicTests(unittest.TestCase):

    def testcase1(self):
        """football, no air resistance"""
        football = t.Tr()
        football.init_pos = [0, 0]
        football.init_vel = [20, 20]

        # position
        self.assertTrue(football.pos(3)[0] == 60)
        self.assertAlmostEqual(football.pos(3)[1], 15.90, delta=0.01)

        # velocity
        self.assertTrue(football.v(3)[0] == 20)
        self.assertAlmostEqual(football.v(3)[1], -9.40, delta=0.01)

        # speed
        self.assertAlmostEqual(football.speed(3), 22.10, delta=0.01)

        # landing point
        x1 = football.landing_point()[0]
        t1 = football.landing_point()[2]
        self.assertAlmostEqual(x1, 81.63, delta=0.01)
        self.assertAlmostEqual(t1, 4.08, delta=0.01)

        # compatibility
        self.assertAlmostEqual(football.i_vel(x1, 45), (2*20**2)**0.5, delta=0.01)
        self.assertAlmostEqual(x1, football.pos(t1)[0], delta=0.01)
        self.assertAlmostEqual(football.pos(t1)[1], 0, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1), t1, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1), football.tot_time(), delta=0.01)

        # max altitude
        x2 = football.max_alt()[0]
        y2 = football.max_alt()[1]
        t2 = football.max_alt()[2]
        self.assertAlmostEqual(y2, 20.41, delta=0.01)
        self.assertAlmostEqual(x2, football.pos(t2)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2), t2, delta=0.01)

        # set_vel_trig
        football.set_vel_trig(10, 30)
        self.assertAlmostEqual(football.init_vel[0], 8.66025, delta=0.0001)
        self.assertAlmostEqual(football.init_vel[1], 5, delta=0.0001)

    def testcase2(self):
        """football, air resistance"""
        # answers are compared with results from Mathematica
        football = t.Tr()
        football.init_pos = [0, 0]
        football.init_vel = [20, 20]
        football.area = 0.038
        football.mass = 0.45
        football.c = 0.25

        # position
        self.assertAlmostEqual(football.pos(3, True)[0], 42.766, delta=0.01)
        self.assertAlmostEqual(football.pos(3, True)[1], 6.235, delta=0.01)

        # velocity
        self.assertAlmostEqual(football.v(3, True)[0], 10.585, delta=0.01)
        self.assertAlmostEqual(football.v(3, True)[1], -11.902, delta=0.01)

        # speed
        self.assertAlmostEqual(football.speed(3, True), 15.930, delta=0.01)

        # landing point
        x1 = football.landing_point(True)[0]
        t1 = football.landing_point(True)[2]
        self.assertAlmostEqual(x1, 47.415, delta=0.01)
        self.assertAlmostEqual(t1, 3.461, delta=0.01)

        # compatibility
        self.assertAlmostEqual(football.i_vel(x1, 45, True, tol=0.0001), (2*20**2)**0.5, delta=0.01)
        self.assertAlmostEqual(x1, football.pos(t1, True)[0], delta=0.01)
        self.assertAlmostEqual(football.pos(t1, True)[1], 0, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1, True), t1, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1, True), football.tot_time(True), delta=0.01)

        # max altitude
        x2 = football.max_alt(True)[0]
        y2 = football.max_alt(True)[1]
        t2 = football.max_alt(True)[2]
        self.assertAlmostEqual(y2, 14.786, delta=0.01)
        self.assertAlmostEqual(x2, football.pos(t2, True)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2, True)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2, True), t2, delta=0.01)

    def testcase3(self):
        """football thrown downwards, air resistance"""
        # answers are compared with results from Mathematica
        football = t.Tr()
        football.init_pos = [-25, 10]
        football.init_vel = [15, -5]
        football.area = 0.038
        football.mass = 0.45
        football.c = 0.25

        # position
        self.assertAlmostEqual(football.pos(3, True)[0], 7.679, delta=0.01)
        self.assertAlmostEqual(football.pos(3, True)[1], -35.878, delta=0.01)

        # velocity
        self.assertAlmostEqual(football.v(3, True)[0], 7.152, delta=0.01)
        self.assertAlmostEqual(football.v(3, True)[1], -22.585, delta=0.01)

        # speed
        self.assertAlmostEqual(football.speed(3, True), 23.69, delta=0.01)

        # landing point
        x1 = football.landing_point(True)[0]
        t1 = football.landing_point(True)[2]
        self.assertAlmostEqual(x1, -10.61, delta=0.01)
        self.assertAlmostEqual(t1, 1.069, delta=0.01)

        # compatibility
        self.assertAlmostEqual(football.i_vel(x1, -18.43494882, True, tol=0.0001), (15**2+5**2)**0.5, delta=0.01)
        self.assertAlmostEqual(x1, football.pos(t1, True)[0], delta=0.01)
        self.assertAlmostEqual(football.pos(t1, True)[1], 0, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1, True), t1, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1, True), football.tot_time(True), delta=0.01)

        # max altitude
        x2 = football.max_alt(True)[0]
        y2 = football.max_alt(True)[1]
        t2 = football.max_alt(True)[2]
        self.assertAlmostEqual(y2, 10, delta=0.01)
        self.assertAlmostEqual(x2, football.pos(t2, True)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2, True)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2, True), t2, delta=0.01)

    def testcase4(self):
        """dropped football, air resistance"""
        # answers are compared with results from Mathematica
        football = t.Tr()
        football.init_pos = [0, 10]
        football.init_vel = [0, 0]
        football.area = 0.038
        football.mass = 0.45
        football.c = 0.25

        # position
        self.assertAlmostEqual(football.pos(3, True)[0], 0, delta=0.0001)
        self.assertAlmostEqual(football.pos(3, True)[1], -27.71, delta=0.01)

        # velocity
        self.assertAlmostEqual(football.v(3, True)[0], 0, delta=0.0001)
        self.assertAlmostEqual(football.v(3, True)[1], -21.80, delta=0.01)

        # speed
        self.assertAlmostEqual(football.speed(3, True), 21.80, delta=0.01)

        # landing point
        x1 = football.landing_point(True)[0]
        t1 = football.landing_point(True)[2]
        self.assertAlmostEqual(x1, 0, delta=0.0001)
        self.assertAlmostEqual(t1, 1.45, delta=0.01)

        # compatibility
        self.assertAlmostEqual(football.i_vel(x1, -90, True, tol=0.0001), 0, delta=0.01)
        self.assertAlmostEqual(x1, football.pos(t1, True)[0], delta=0.01)
        self.assertAlmostEqual(football.pos(t1, True)[1], 0, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1, True), 0, delta=0.01)
        self.assertAlmostEqual(football.tot_time(True), t1, delta=0.01)

        # max altitude
        x2 = football.max_alt(True)[0]
        y2 = football.max_alt(True)[1]
        t2 = football.max_alt(True)[2]
        self.assertAlmostEqual(y2, 10, delta=0.01)
        self.assertAlmostEqual(x2, football.pos(t2, True)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2, True)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2, True), t2, delta=0.01)

    def testcase5(self):
        """dropped football, no air resistance"""
        football = t.Tr()
        football.init_pos = [0, 10]
        football.init_vel = [0, 0]
        football.area = 0.038
        football.mass = 0.45
        football.c = 0.25

        # position
        self.assertAlmostEqual(football.pos(3)[0], 0, delta=0.0001)
        self.assertAlmostEqual(football.pos(3)[1], -34.10, delta=0.01)

        # velocity
        self.assertAlmostEqual(football.v(3)[0], 0, delta=0.0001)
        self.assertAlmostEqual(football.v(3)[1], -29.40, delta=0.01)

        # speed
        self.assertAlmostEqual(football.speed(3), 29.40, delta=0.01)

        # landing point
        x1 = football.landing_point()[0]
        t1 = football.landing_point()[2]
        self.assertAlmostEqual(x1, 0, delta=0.0001)
        self.assertAlmostEqual(t1, 1.429, delta=0.01)

        # compatibility
        self.assertAlmostEqual(football.i_vel(x1, -90, False, tol=0.0001), 0, delta=0.01)
        self.assertAlmostEqual(x1, football.pos(t1)[0], delta=0.01)
        self.assertAlmostEqual(football.pos(t1)[1], 0, delta=0.01)
        self.assertAlmostEqual(football.time_x(x1), 0, delta=0.01)
        self.assertAlmostEqual(football.tot_time(), t1, delta=0.01)

        # max altitude
        x2 = football.max_alt()[0]
        y2 = football.max_alt()[1]
        t2 = football.max_alt()[2]
        self.assertAlmostEqual(y2, 10, delta=0.01)
        self.assertAlmostEqual(x2, football.pos(t2)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2), t2, delta=0.01)

    def testcase6(self):
        """edge cases"""
        # answers are compared with results from Mathematica
        football = t.Tr()
        football.init_pos = [0, -10]
        football.init_vel = [0, 0]
        football.area = 0.038
        football.mass = 0.45
        football.c = 0.25

        # impossible landing point
        try:
            assert football.landing_point()[0]
        except RuntimeError:
            pass
        try:
            assert football.landing_point(True)[0]
        except RuntimeError:
            pass
        try:
            assert football.i_vel(10, -90, False, tol=0.0001)
        except RuntimeError:
            pass

        try:
            assert football.i_vel(10, -90, True, tol=0.0001)
        except RuntimeError:
            pass

        # shots from pit
        self.assertAlmostEqual(football.i_vel(20, 70, False, tol=0.0001), 19.31, delta=0.01)
        self.assertAlmostEqual(football.i_vel(-20, 100, False, tol=0.0001), 25.07, delta=0.01)

        # max altitude
        x2 = football.max_alt()[0]
        y2 = football.max_alt()[1]
        t2 = football.max_alt()[2]
        self.assertAlmostEqual(y2, -10, delta=0.0001)
        self.assertAlmostEqual(x2, football.pos(t2)[0], delta=0.01)
        self.assertAlmostEqual(y2, football.pos(t2)[1], delta=0.01)
        self.assertAlmostEqual(football.time_x(x2), t2, delta=0.01)

if __name__ == '__main__':
    unittest.main()