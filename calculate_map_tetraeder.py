import math
import numpy as np
import matplotlib.pyplot as plt


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
LED_START_OFFSET = 1 # base units
LED_PER_EDGE = 5
EDGE_END = LED_PER_EDGE - 1
THETA_TO_Z = (90-53.74) # thet down from z axis for non-flat edges
THETA_FLAT = 90

print("Starting calculations...")
edges =[]

edge_config =[ # base vector indexes and angles of edge, (angles azimut and polar) [edge index,led_index, theta, phi]
    # edge, led, azimut theta from top z-axis, polar phi in direction of x-axis
    [0, 0, THETA_FLAT, 0], # 0
    [0, 0, THETA_FLAT, 60], # 1
    [0, 0, THETA_TO_Z, 30], # 2
    
    [0, EDGE_END, THETA_TO_Z, 90+30+30], # 3
    [0, EDGE_END, THETA_FLAT, 90+30], # 4
    [1, EDGE_END, THETA_TO_Z, 90+90+90], # 5

    # [0, (LED_PER_EDGE-1), 20, 92],
    # [0, (LED_PER_EDGE-1), 20, 93],
    # [0, (LED_PER_EDGE-1), 20, 94],
    # [0, (LED_PER_EDGE-1), 20, 95],

    # [0, 0, 20, 90], # 10
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],

    # [0, 0, 20, 90], # 15
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],

    # [0, 0, 20, 90], # 20
    # [0, 0, 20, 90],
    # [0, 0, 20, 90],
    # [0, 0, 20, 90], # 23
]
for edge_index, curr_edge in enumerate(edge_config):
    edges.append([]) # add new edge to leds list

    # get edge config from list
    base_edge_index, base_led_index, theta, phi = curr_edge
    # check for start vector (bootstrap) or lookup
    if base_led_index == 0 and base_edge_index == 0: 
        base_vector = np.array([0,0,0])
    else:
        base_vector = edges[base_edge_index][base_led_index]
    
    # add LEDs
    for curr_led in range(LED_PER_EDGE):
        a = LED_START_OFFSET + (curr_led* LED_PITCH)
        x = a*math.sin((theta/180*math.pi))*math.cos((phi/180*math.pi))
        y = a*math.sin((theta/180*math.pi))*math.sin((phi/180*math.pi))
        z = a*math.cos((theta/180*math.pi))
        led_vector_local = np.array([x,y,z])
        led = led_vector_local + base_vector
        edges[edge_index].append(led)
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

for edge in edges:
    for led in edge:
        ax.scatter(led[0], led[1], led[2], marker='o')

plt.show()