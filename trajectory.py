# "Short range ballistic trajectories" is a library that let's you compute different properties of the ballistic
# trajectory of a free rigid body in motion.
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp as solve

g = 9.8
p = 1.204  # kg/m³ density of air at NTP

class Tr:
    """Compute different properties of the ballistic trajectory of a free body in motion. SI-units are assumed."""

    def __init__(self):
        """Initialize a body as an object with properties at time t=0."""
        self.init_pos = [0, 0]
        self.init_vel = [0, 0]
        self.area = None
        self.mass = None
        self.c = None

    def __drag(self):
        v = self.init_vel
        try:
            f = 0.5*p*self.c*self.area
        except:
            f = 0
        if f == 0:
            raise RuntimeError("Reference area, mass and drag coefficient have to be provided to "
                               "perform this operation")
        return f

    def __ode_solver(self, t):
        c = Tr.__drag(self)
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = self.init_vel[0]
        v0y = self.init_vel[1]
        r = [v0x, x0, v0y, y0]
        m = self.mass
        f = lambda t, r: [-c/m*r[0]*(r[0]**2+r[2]**2)**0.5, r[0],-c/m*r[2]*(r[0]**2+r[2]**2)**0.5-g, r[2]]
        return solve(f, [0, t], r)

    def __ode_solver_impact(self, v0, t_max):
        c = Tr.__drag(self)
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = v0[0]
        v0y = v0[1]
        r = [v0x, x0, v0y, y0]
        m = self.mass
        f = lambda t, r: [-c/m*r[0]*(r[0]**2+r[2]**2)**0.5, r[0],-c/m*r[2]*(r[0]**2+r[2]**2)**0.5-g, r[2]]
        hit = lambda t, r: r[3]
        hit.terminal = True
        hit.direction = -1
        return solve(f, [0, t_max], r, events=hit)

    def __ode_solver_reach_x(self, x, v0, t_max):
        c = Tr.__drag(self)
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = v0[0]
        v0y = v0[1]
        r = [v0x, x0, v0y, y0]
        m = self.mass
        f = lambda t, r: [-c/m*r[0]*(r[0]**2+r[2]**2)**0.5, r[0],-c/m*r[2]*(r[0]**2+r[2]**2)**0.5-g, r[2]]
        hit = lambda t, r: x - r[1]  # 0 at goal
        hit.terminal = True
        return solve(f, [0, t_max], r, events=hit)

    def __ode_solver_max_y(self, v0, t_max):
        c = Tr.__drag(self)
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = v0[0]
        v0y = v0[1]
        r = [v0x, x0, v0y, y0]
        m = self.mass
        f = lambda t, r: [-c/m*r[0]*(r[0]**2+r[2]**2)**0.5, r[0],-c/m*r[2]*(r[0]**2+r[2]**2)**0.5-g, r[2]]
        hit = lambda t, r: r[2]  # 0 at goal
        hit.terminal = True
        return solve(f, [0, t_max], r, events=hit)

    def pos(self, t, air=False):
        """Return position of this body at time t as a list/vector [x, y]. Set air=True to apply air resistance."""
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = self.init_vel[0]
        v0y = self.init_vel[1]
        if not air:
            y = -g * t ** 2 / 2 + v0y * t + y0
            x = v0x * t + x0
            p = [x, y]
        else:
            sol = Tr.__ode_solver(self, t)
            last = len(sol.t)-1
            x = sol.y[1][last]
            y = sol.y[3][last]
            p = [x, y]
        return p

    def v(self, t, air=False):
        """Return velocity of this body at time t as a list/vector [vx, vy]. Set air=True to apply air resistance."""
        if not air:
            v0x = self.init_vel[0]
            v0y = self.init_vel[1]
            vy = -g * t + v0y
            vx = v0x
        else:
            sol = Tr.__ode_solver(self, t)
            last = len(sol.t) - 1
            vx = sol.y[0][last]
            vy = sol.y[2][last]
        return [vx, vy]

    def speed(self, t, air=False):
        """Return speed of this body at time t as a scalar. Set air=True to apply air resistance."""
        v = Tr.v(self, t, air)
        return (v[1]**2 + v[0]**2)**0.5

    def i_vel(self, x, a, air=False, tol=0.001):
        """Return necessary initial speed at angle a in degrees for this body to land at a distance x as a scalar.
        Set air=True to apply air resistance. Set a lower tolerance tol to get a more precise answer.
        RuntimeError is raised if no solution is found."""
        a = a*math.pi/180
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = abs(x - x0) * (1 / math.tan(a)) * (g * math.tan(a) / (2 * x - 2 * x0 + 2 * y0 * (1 / math.tan(a))))**0.5
        v0y = abs(x - x0) / (2 * (1 / math.tan(a)) * (x - x0 + y0 * (1 / math.tan(a))) / g) ** 0.5
        q1 = x0*math.tan(a)
        q2 = y0 + x*math.tan(a)
        v0 = [v0x, v0y]  # solution for no air resistance
        if air:
            tol = abs(tol*x)
            error_c = 2*tol
            a = -10
            b = 10
            c = 0
            i = 0
            while abs(error_c) > tol:
                c = (a+b)/2
                xc_result = Tr.landing_point(self, True, [c*v0x, c*v0y])[0]
                error_c = x - xc_result
                if error_c == 0:
                    break
                xa_result = Tr.landing_point(self, True, [a*v0x, a*v0y])[0]
                error_a = x - xa_result
                if error_a*error_c < 0:
                    b = c
                else:
                    a = c
                i += 1
            v0 = [c*v0x, c*v0y]
        v = (v0[0]**2 + v0[1]**2)**0.5
        if type(v) is complex:  # body never lands
            raise RuntimeError("No real solution was found.")
        return v

    def set_vel_trig(self, speed, angle):
        """Change the initial velocity vector of this body by providing speed and angle (in °) as two scalars.
        Return None."""
        a = angle*math.pi / 180
        v0x = speed*math.cos(a)
        v0y = speed*math.sin(a)
        self.init_vel = [v0x, v0y]
        return None

    def max_alt(self, air=False):
        """Return position and time of this body at it's highest altitude as a list [x, y, t].
        Set air=True to apply air resistance."""
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        v0x = self.init_vel[0]
        v0y = self.init_vel[1]

        if v0y <= 0:  # body is dropped
            t = 0
            p = Tr.pos(self, 0)
        else:
            tmax = v0y/g
            if not air:
                p = Tr.pos(self, tmax)
                t = tmax
            else:
                sol = Tr.__ode_solver_max_y(self, [v0x, v0y], tmax * 10)
                if len(sol.t_events[0]) == 0:  # no solution was found
                    return None
                last = len(sol.t) - 1
                t = sol.t[last]
                x = sol.y[1][last]
                y = sol.y[3][last]
                p = [x, y]
        p.append(t)
        return p

    def landing_point(self, air=False, v0=None):
        """Return landing point and moment of impact for this body as a list [x, y, t].
        Set air=True to apply air resistance. Override initial velocity of this body by providing a vector v0=[vx, vy].
        RuntimeError is raised if no solution is found."""
        x0 = self.init_pos[0]
        y0 = self.init_pos[1]
        if not v0:
            v0x = self.init_vel[0]
            v0y = self.init_vel[1]
        else:
            v0x = v0[0]
            v0y = v0[1]
        t_imp_no_air = v0y / g + ((v0y / g) ** 2 + 2 * y0 / g) ** 0.5  # only biggest solution
        if t_imp_no_air == 0:
            return [0, 0, 0]
        if type(t_imp_no_air) is complex:  # body never lands
            raise RuntimeError("No real solution was found. It seems like this body never lands.")
        if not air:
            x = Tr.pos(self, t_imp_no_air)[0]
            return [x, 0, t_imp_no_air]
        else:
            sol = Tr.__ode_solver_impact(self, [v0x, v0y], t_imp_no_air*10)
            if len(sol.t_events[0]) == 0:  # no solution was found
                return None
            last = len(sol.t)-1
            x = sol.y[1][last]
            t = sol.t[last]
            return [x, 0, t]

    def time_x(self, x, air=False):
        """Return time for this body to reach a distance x as a scalar. Set air=True to apply air resistance.
        Return None if no solution is found."""
        x0 = self.init_pos[0]
        v0x = self.init_vel[0]
        v0y = self.init_vel[1]
        if v0x != 0:
            t = (x - x0) / v0x
            if t == 0:
                return 0
            elif t < 0:
                return None
            if air:
                sol = Tr.__ode_solver_reach_x(self, x, [v0x, v0y], t*10)
                if not sol.t_events[0]:  # no solution was found
                    return None
                last = len(sol.t) - 1
                t = sol.t[last]
        else:
            t = 0
        return t

    def tot_time(self, air=False):
        """Return total time of flight of this body. Set air=True to apply air resistance."""
        p = Tr.landing_point(self, air)
        return p[2]

    def add_plot(self, t, air=False, c="red", res=100):
        """Add a plot of the trajectory of this body from time t=0 to t. Set air=True to apply air resistance.
        Set line color with argument c as a string. Set a higher resolution res to get a more precise result.
        Call show_plot() to show the plot. Return None."""
        h = t/res
        t_val = []
        i = 0
        while i <= t:
            t_val.append(i)
            i += h
        x_points = []
        y_points = []
        for j in t_val:
            p = Tr.pos(self, j, air)
            x_points.append(p[0])
            y_points.append(p[1])

        plt.plot(x_points, y_points, linewidth=1, color=c)
        plt.title("Trajectory from t=0 to t")
        plt.xlabel("x")
        plt.ylabel("y")
        return None

    def show_plot(self):
        """Show plots added with add_plot. Return None."""
        plt.show()
        return None