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

theta = 0

print("Starting calculations...")
edges =[]
for curr_edge in range(24):
    edges.append([]) # add new edge leds list

    # look up config of edge here
    base_vector = np.array([0,0,0])
    theta += 20 / 180 * math.pi
    phi = 90 / 180 * math.pi

    #add LEDs
    for curr_led in range(59):
        a = LED_START_OFFSET + (curr_led* LED_PITCH)
        x = a*math.sin(theta)*math.cos(phi)
        y = a*math.sin(theta)*math.sin(phi)
        z = a*math.cos(theta)
        led = [x,y,z]
        edges[curr_edge].append(led)
        #print(x,y,z)
        

# debug plotting
print("Starting debug plot...")
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

for edge in edges:
    for led in edge:
        ax.scatter(led[0], led[1], led[2], marker='o')

plt.show()