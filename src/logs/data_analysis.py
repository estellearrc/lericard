import numpy as np
import matplotlib.pyplot as plt
import sys

if __name__ == "__main__":
    file_name = str(sys.argv[1])
    with open(file_name, "r") as f:
        mission_name = str(f.readline().split("\n")[0])
        variables = f.readline().split(",")
        variables.pop()
        values = {}
        for key in variables:
            values[key] = []
        
        for line in f.readlines():
            for i in range(len(variables)):
                values[variables[i]].append(float(line.split(",")[i]))

    if mission_name == "triangle":
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle('Heading and motor commands : mission ' + mission_name)
        ax1.plot(values['t'], values['heading'], 'b-', label = 'heading route')
        ax1.plot(values['t'], values['heading_obj'], 'k--', label = 'heading objective')
        
        
        ax2.plot(values['t'], values['u_L'], 'r-', label = 'left command')
        ax2.plot(values['t'], values['u_R'], 'g-', label = 'right command')
        ax1.set(xlabel = 'Time (s)', ylabel = 'Headings (rad)')
        ax2.set(xlabel = 'Time (s)', ylabel = 'Motors commands (0-255)')
        fig.legend()
        
        plt.figure(2)
        plt.plot(values['pos_x'], values['pos_y'], 'b.')
        plt.title("GPS route : mission " + mission_name)
        plt.xlabel('Position x (m)')
        plt.ylabel('Position y (m)')
        plt.legend()    
    
    elif mission_name == 'goNorth':
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle('Heading and motor commands : mission ' + mission_name)
        ax1.plot(values['heading'], 'b-', label = 'heading route')
        ax2.plot(values['u_L'], 'r-', label = 'left command')
        ax2.plot(values['u_R'], 'g-', label = 'right command')
        ax2.set(ylabel = 'Motors commands (0-255)')
        ax1.set(ylabel = 'Heading objective (rad)')
        fig.legend()
    
    elif mission_name == "test_GPS":
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle('Heading and speed : mission ' + mission_name)
        ax1.plot(values['t'], values['speed'], 'b-', label = 'heading route')
        
        
        ax2.plot(values['t'], values['heading'], 'r-', label = 'left command')
        ax1.set(xlabel = 'Time (s)', ylabel = 'Speed (m/s)')
        ax2.set(xlabel = 'Time (s)', ylabel = 'Heading (rad)')
        fig.legend()
        
        plt.figure(2)
        plt.plot(values['lat'], values['long'], 'b.')
        plt.title("GPS route : mission " + mission_name)
        plt.xlabel('Position x (m)')
        plt.ylabel('Position y (m)')
        plt.legend()    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        

        
    plt.show()
