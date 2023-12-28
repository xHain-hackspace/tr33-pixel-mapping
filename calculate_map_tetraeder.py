import math
import numpy as np
import matplotlib.pyplot as plt

def vector_from_angles(magnitude, theta, phi):
        """Create a vector from magnitude and directon angles"""
        a = magnitude
        x = a*math.sin((theta/180*math.pi))*math.cos((phi/180*math.pi))
        y = a*math.sin((theta/180*math.pi))*math.sin((phi/180*math.pi))
        z = a*math.cos((theta/180*math.pi))
        return np.array([x,y,z])


#Komponenten eines Vektors a⃗  aus Betrag a und den Winkeln α, β und γ zu den Koordinatenachsen:
#x-Komponente: ax=acos(α)
#y-Komponente: ay=acos(β)
#z-Komponente: az=acos(γ)
#Komponenten eines Vektors a⃗ 
#aus Betrag a und den Azimutalwinkel φ und dem Polarwinkeln θ in Kugelkoordinate:
#x-Komponente: ax=asin(θ)cos(φ)
#y-Komponente: ay=asin(θ)sin(φ)
#z-Komponente: az=acos(θ)

LED_PITCH = 1 # base units
CAP_TIP_OFFSET = 1 # base units, cap tip to first/last LED
LED_PER_EDGE = 5
EDGE_END = LED_PER_EDGE - 1
THETA_TO_Z = (90-53.74) # theta down from z axis for non-flat edges
THETA_FLAT = 90
FULL_EDGE_LENGTH =  CAP_TIP_OFFSET*2 + (LED_PITCH * LED_PER_EDGE) # full geometrical edge length including caps
ORIGIN = np.array([0, 0, 0]) # zero vector
print("Starting calculations...")
led_coordinates =[]

# config of the basic shape, e.g. one sub-tetraeder
EDGE0_END = vector_from_angles(FULL_EDGE_LENGTH, THETA_FLAT, 0)
EDGE1_END = vector_from_angles(FULL_EDGE_LENGTH, THETA_FLAT, 60)
base_shape_edge_config =[ # base vector pointing to edge start, angles azimut and polar [npa.array vector, theta, phi]
    # edge, led, azimut theta from top z-axis, polar phi in direction of x-axis
    [ORIGIN, THETA_FLAT, 0], # 0
    [ORIGIN, THETA_FLAT, 60], # 1
    [ORIGIN, THETA_TO_Z, 30], # 2
    
    [EDGE0_END, THETA_TO_Z, 90+30+30], # 3
    [EDGE0_END, THETA_FLAT, 90+30], # 4
    [EDGE1_END, THETA_TO_Z, 90+90+90], # 5
]

# how to replicate/shift the basic shape. consists of base vector pointing to the shifted shape origin
replication_config = [
    np.array([0,0,0]),
    vector_from_angles(FULL_EDGE_LENGTH, THETA_TO_Z, 30),
]
for replica_index, replica_base_vector in enumerate(replication_config): # iterate over all shape replicas
    for edge_index, curr_edge in enumerate(base_shape_edge_config): # for every edge of the base shape edges
        led_coordinates.append([]) # add new edge to leds global list

        # get edge config from base shape config
        edge_base_vector, theta, phi = curr_edge 

        nr_of_base_edges = len(base_shape_edge_config)
        global_index = (replica_index * nr_of_base_edges) + edge_index
        
        # add LEDs
        for curr_led in range(LED_PER_EDGE):
            magnitude = CAP_TIP_OFFSET + (curr_led* LED_PITCH)
            led_vector_local = vector_from_angles(magnitude, theta, phi)
            led_vector = led_vector_local + edge_base_vector + replica_base_vector
            led_coordinates[global_index].append(led_vector)
            #print(x,y,z)
        

# debug plotting
print("Starting debug plot...")
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.axes.set_xlim3d(left=0, right=10) 
ax.axes.set_ylim3d(bottom=0, top=10) 
ax.axes.set_zlim3d(bottom=0, top=10) 

for edge in led_coordinates:
    for led_vector in edge:
        ax.scatter(led_vector[0], led_vector[1], led_vector[2], marker='o')

plt.show()