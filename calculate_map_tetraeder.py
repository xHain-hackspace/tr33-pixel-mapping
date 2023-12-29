import math
import numpy as np
import matplotlib.pyplot as plt #too slow
import plotly.graph_objects as go

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

LED_PITCH = 1/60 # meter
CAP_TIP_OFFSET = 6/100 # meter, cap tip to first/last LED
LED_PER_EDGE = 59
THETA_TO_Z = (90-53.74) # theta down from z axis for non-flat edges
THETA_FLAT = 90
FULL_EDGE_LENGTH =  CAP_TIP_OFFSET*2 + (LED_PITCH * LED_PER_EDGE) # full geometrical edge length including caps
ORIGIN = np.array([0, 0, 0]) # zero vector
SCALE_FACTOR = 1/2*8 # scale 2m to 8
print("Starting calculations...")
edges =[]
edges_global_index = -1

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
    vector_from_angles(FULL_EDGE_LENGTH, THETA_FLAT, 0),
    vector_from_angles(FULL_EDGE_LENGTH, THETA_FLAT, 60),
]
for replica_index, replica_base_vector in enumerate(replication_config): # iterate over all shape replicas
    for edge_index, curr_edge in enumerate(base_shape_edge_config): # for every edge of the base shape edges
        # prepare new edge slot
        edges.append([])
        edges_global_index += 1
        # get edge config from base shape config
        edge_base_vector, theta, phi = curr_edge 
        
        # add LEDs
        for curr_led in range(LED_PER_EDGE):
            magnitude = CAP_TIP_OFFSET + (curr_led* LED_PITCH)
            led_vector_local = vector_from_angles(magnitude, theta, phi)
            led_vector = led_vector_local + edge_base_vector + replica_base_vector
            led_vector = SCALE_FACTOR * led_vector
            edges[edges_global_index].append(led_vector)
            #print(x,y,z)
                    

# debug plotting
print("Starting debug plot...")

# # matplotlib is slow
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.axes.set_xlim3d(left=0, right=10) 
# ax.axes.set_ylim3d(bottom=0, top=10) 
# ax.axes.set_zlim3d(bottom=0, top=10) 

# for curr_edge in edges:
#     for led_vector in curr_edge:
#         ax.scatter(led_vector[0], led_vector[1], led_vector[2], marker='o')

# plt.show()

fig = go.Figure(data =[go.Scatter3d(mode ='markers')])

# this is probably not efficient: assemble x,y,z vectors per edge and add to plot
edge_nr = -1
sourcecode = "{ "
for curr_edge in edges:
    edge_nr += 1
    led_nr = -1
    xi = []
    yi = []
    zi = []    
    for led_vector in curr_edge:
        led_nr += 1
        # get components
        x = led_vector[0]
        y = led_vector[1]
        z = led_vector[2]
        # add to vectors for debug plot
        xi.append(x)
        yi.append(y)
        zi.append(z)
        # add sourcecode line
        sourcecode += f"{{{edge_nr}, {led_nr}, {x:.6f}, {y:.6f}, {z:.6f}}},\n"
    
    # add points for every edge
    fig.add_trace(
        go.Scatter3d(x= xi,
                    y= yi,
                    z= zi,
                    mode='markers',
                    marker=dict(
                        size=0.9,
                        color= edge_nr,  # set color to an array/list of desired values
                        opacity=1
                    )
        )
    )
# show
fig.show()
# maybe remove last comma here
sourcecode += " }\n"
#print(sourcecode)
with open("mapping.txt", "w") as text_file:
    text_file.write(sourcecode)