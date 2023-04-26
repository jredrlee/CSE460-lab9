import socket
import sys
import time
import math
import networkx as nx
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

IP_ADDRESS = "192.168.0.209" #'192.168.0.202'

positions = {}
rotations = {}

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz




if __name__ == "__main__":
    clientAddress = "192.168.0.132"
    optitrackServerAddress = "192.168.0.172"
    robot_id = 209

    
    #robot_control_code 
    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')


    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    #Initial Parameters

    #Need 10 vertices: a,b,c,d,e,f,g, h, i, j
    
    a = (-3.87542, 3.18673, 0.0)
    b = (-2.36880, 3.14329, 0.0)
    c = (-1.87048, 2.55707, 0.0)
    d = (-2.50179, 2.55706, 0.0)
    e = (-1.57988, 1.95678, 0.0)
    f = (-1.46608, 1.21648, 0.0)
    g = (-2.05683, 0.54819, 0.0)
    h = (-3.88985, 1.87119, 0.0)
    i = (-3.31824, 1.36641, 0.0)
    j = (-3.10237, 0.65406, 0.0)
    
    
   
    #starting and ending point
    starting = a
    ending = j
    
    # Make the nx graph and add the vertices(nodes) and their edges
    graph = nx.Graph()
    #graph.add_nodes_from([a, b,c, d, e, f, g, h, i, j])
    graph.add_nodes_from([
        (a, {"label": "a"}), 
        (b, {"label": "b"}),
        (c, {"label": "c"}), 
        (d, {"label": "d"}), 
        (e, {"label": "e"}), 
        (f, {"label": "f"}), 
        (g, {"label": "f"}), 
        (h, {"label": "h"}), 
        (i, {"label": "i"}), 
        (j, {"label": "j"})])
    graph.add_edges_from([ (a,b), (a,h), (b,c), (b,d), (c,d), (c,e), (e,d), (e,f), (f,g), (g,j), (d,h), (d,i), (h,i), (i, j)], weight = 0.0)
    #print(graph.number_of_nodes())
    #print(graph.number_of_edges())


    #graph.edges.data()
    #change the weight of the edges to equal the distance between the nodes
    #for i in graph.edges.data():
        #graph.add_edge(i[0], i[1], weight = math.dist(i[0], i[1]))
    


        
    path = nx.shortest_path(graph, source = j,target = b,  method = 'dijkstra')
    print(path)

    
    #####
    midterm_time = 0.0
    total_time = 0.0
    robot_data = []
    destination_data = []
    robot_time = []
    done = False
    try:
        #midterm_time = 0.0
        degree = 180/math.pi
        path_length = len(path)
        path_counter = 0
        
        while is_running:
            if (done == True):
                break
            else:

                if robot_id in positions:
                
                    #P##############################################################################

                    ##### last position
                    print('Last position', positions[robot_id], ' rotation', rotations[robot_id])

                    #Equation 3.2 from notes with tf = 5.0
                    # y = (a1 -a0)/time + a0

                    #difference between the two positions in path
                    pos_diff1 = tuple(map(lambda i, j: i - j, path[path_counter + 1], path[path_counter]))
                    #divide the values of the pos_diff1 by 5.0
                    pos_diff2 = tuple(ti/10.0 for ti in pos_diff1)

                    Goal = tuple(map(lambda i, j: i + j, pos_diff2, path[path_counter]))
                    
                    #pos_difference = path[path_counter + 1] - path[path_counter]
                    #pos_difference = tuple(map(lambda i, j: i - j, path[path_counter + 1], path[path_counter]))
                    #Goal = (pos_difference)/2.5 + path[path_counter]


                    #Straight-Line Trajectory
                    res = tuple(map(lambda i, j: i - j, path[path_counter + 1], Goal))

                    #res2 is for difference between current location and end goal
                    res2 = tuple(map(lambda i, j: i - j, path[path_counter + 1], positions[robot_id]))
                    #prints distance between goal and robot location
                    print('distance', res ) 

                    ####arc-tangent equation
                    
                    alpha = (math.atan2(res[1], res[0]) *180) / math.pi
                    print('alpha ', alpha)

                    #omega = 300 *  (alpha - rotations[robot_id])
                    omega = 80 * ((math.atan2(math.sin((alpha - rotations[robot_id])*(math.pi / 180)), math.cos((alpha - rotations[robot_id])* (math.pi/180))) * 180) / math.pi)
                    
                    
                    v = 1200 * math.sqrt((res2[1] ** 2) + (res2[0]**2))
                    #v = 1400
                    u = np.array([v - omega, v + omega])

                    u[u > 1500] = 1500
                    u[u < -1500] = -1500

                    #### Send control input to the motors
                    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                    s.send(command.encode('utf-8'))

                    midterm_time += 0.1
                    total_time += 0.1
                    print(midterm_time)
                    if (midterm_time > 6.0):
                        midterm_time = 0
                        path_counter +=1
                        if (path_counter > path_length -2): # all the coordinates have been visited
                            done = True
                            break
                            
                    time.sleep(0.1) 
    
        command = 'CMD_MOTOR#00#00#00#00\n'
        print("Time: ",total_time)
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        
        s.close()
    except KeyboardInterrupt:
        # STOP
        print("Not working")
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        streaming_client.shutdown()
        
    
    s.close()


#####################################

