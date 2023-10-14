import trajectory
# This example demonstrates how to úse the module "trajectory" to plot the trajectory of a soccer ball
football = trajectory.Tr()
football.init_pos = [0, 0]
football.set_vel_trig(20, 45)  # shot at initial speed 20 m/s at initial angle 45°

# Without air resistance:
impact_time_1 = football.landing_point()[2]
football.add_plot(impact_time_1)

# With air resistance:
football.area = 0.038
football.mass = 0.45
football.c = 0.25
impact_time_2 = football.landing_point(True)[2]
football.add_plot(impact_time_2, True, "blue")

football.show_plot()
