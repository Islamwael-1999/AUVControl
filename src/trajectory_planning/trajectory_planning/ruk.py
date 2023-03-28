from pathlib import Path
from sys import path
from copy import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute().parent / 'build'
path.insert(0, str(build_path))
 
from ruckig import InputParameter, Ruckig, Trajectory, Result,OutputParameter
 
 
if __name__ == '__main__':
    otg = Ruckig(3, 0.001)  # DoFs, control cycle
    out = OutputParameter(3)
    inp = InputParameter(3)

    inp.current_position = [0.0, 0.0, 0.0]
    inp.current_velocity = [0.0, -0.2, -0.1]
    inp.current_acceleration = [0.05, 0.05, -0.05]

    inp.target_position = [1.0, -1.0, -0.785]
    inp.target_velocity = [0.0, 0.0, 0.0]
    inp.target_acceleration = [0.0, 0.0, 0.0]

    inp.max_velocity = [0.3, 0.3, 0.3]
    inp.max_acceleration = [0.1, 0.1, 0.1]
    inp.max_jerk = [0.05, 0.05, 0.05]
 
    # Set different constraints for negative direction
    inp.min_velocity = [-1.0, -0.5, -3.0]
    inp.min_acceleration = [-2.0, -1.0, -2.0]
 
    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))
 
    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
 
        print('\t'.join([f'{out.time:0.3f}'] + [f'{p:0.3f}' for p in out.new_position]))
        out_list.append(copy(out))
 
        out.pass_to_input(inp)
 
        if not first_output:
            first_output = copy(out)
 
    print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
    print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')
    

    # Plot the trajectory
    # path.insert(0, str(Path(__file__).parent.absolute().parent / 'test'))
    # from plotter import Plotter
 
    # Plotter.plot_trajectory(Path(__file__).parent.absolute() / '1_trajectory.pdf', otg, inp, out_list, plot_jerk=False)
    import numpy as np
    posx,posy,posz=[],[],[]
    for out in out_list:
        posx.append(out.new_position[0])
        posy.append(out.new_position[1])
        posz.append(out.new_position[2])
        print(out)
    
    

    ax = plt.axes(projection='3d')
    ax.plot3D(posx,posy, posz, 'blue')

    plt.title("ruckig Curves")
    plt.grid(True)

    plt.show()
    # follow_list = np.array(follow_list)
    # target_list = np.array(target_list)

    # plt.ylabel(f'DoF 1')
    # plt.plot(steps, follow_list[:, 0], label='Follow Position')
    # plt.plot(steps, follow_list[:, 1], label='Follow Velocity', linestyle='dotted')
    # plt.plot(steps, follow_list[:, 2], label='Follow Acceleration', linestyle='dotted')
    # plt.plot(steps, target_list[:, 0], color='r', label='Target Position')
    # plt.grid(True)
    # plt.legend()
 
    # plt.savefig(Path(file).parent.absolute() / '13_trajectory.pdf')