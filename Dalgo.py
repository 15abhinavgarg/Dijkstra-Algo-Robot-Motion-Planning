'''
Name : Abhinav Garg
ENPM-661
Planning for Autonomous Robots
The below code is an intellectual property of Abhinav Garg
'''

import heapq as hq
import numpy as np
import cv2
import matplotlib.pyplot as plt

###### Taking start and Goal coordinate ################

def taking_inputs(img):

    xi = int(input("Enter y coordinate of start point = "))
    yi = int(input("Enter x coordinate of start point = "))

    xg = int(input("Enter y coordinate of GOAL point = "))
    yg = int(input("Enter x coordinate of GOAL point = "))

    if img[xi][yi] == 0:
        print("The start location is on an obstacle kindly re-input the points")
        exit()
    
    if img[xg][yg] == 0:
        print("The goal location is on an obstacle kindly re-input the points")
        exit()
    
    if xi >= 250 or xi <0 :
        print("y coordinate of start point is out of bounds")
        exit()
    
    if xg >= 250 or xg <0 :
        print("y coordinate of goal point is out of bounds")
        exit()
    
    if yi >= 400 or yi < 0:
        print("x coordinate of start point is out of bounds")
        exit()
    
    if yg >= 400 or yg < 0:
        print("x coordinate of goal point is out of bounds")
        exit()
    
    return xi,yi,xg,yg

#######  Drawing obstacles in the Search space ########

def draw_obstacles():
    data = np.zeros((250,400), dtype=np.uint8)
    data[:,:] = [255]
    
    img = cv2.circle(data, (300,65), 40, (0), -1) #drawing a circle as an obstacle

    hex = np.array( [ [200,110], [235,130], [235, 170], [200,190], [165,170], [165,130] ] ) #drawing hexagonal obstacle
    cv2.fillPoly(img, pts =[hex], color=(0))
    
    poly = np.array( [ [36,65], [115,40], [80,70], [105,150] ] ) #drawing polygonal shape as obstacle
    cv2.fillPoly(img, pts =[poly], color=(0))
   
    return img

############  Defining all the Action steps ############
def allpossiblesteps(x,y):
    steps = []
    x_max = 250
    y_max = 400
    if x+1 < x_max:
        c2c = 1
        temp = (x+1,y,c2c)
        steps.append(temp)
        #add to the queue
    
    if x-1 >= 0:
        c2c = 1
        temp = (x-1,y,c2c)
        steps.append(temp)
    
    if y+1 < y_max:
        c2c = 1
        temp = (x,y+1,c2c)
        steps.append(temp)

    if y-1 >= 0:
        c2c = 1
        temp = (x,y-1,c2c)
        steps.append(temp)
    
    if x+1 < x_max and y+1 < y_max: #DR
        c2c = 1.4
        temp = (x+1,y+1,c2c)
        steps.append(temp)

    if x+1 < x_max and y-1 >= 0:  #TR
        c2c = 1.4
        temp = (x+1,y-1,c2c)
        steps.append(temp)
    
    if x-1 >= 0 and y-1 >= 0: #TL
        c2c = 1.4
        temp = (x-1,y-1,c2c)
        steps.append(temp)

    if x-1 >= 0 and y+1 < y_max:  #DL
        c2c = 1.4
        temp = (x-1,y+1,c2c)
        steps.append(temp)
    return steps


def main():

    img = draw_obstacles()

    xi,yi,xg,yg = taking_inputs(img)

    Goal_var = (xg,yg)  #storing goal location
 
    Q = [] #DS to fetch the lowest c2c
    d1 = (0,(-1,-1),(xi,yi))  #initialised -1 as parents of start point
    hq.heappush(Q, d1)
    hq.heapify(Q)
    
    cq = {} #initialising closed list
    
    oq = {}
    oq[(xi,yi)] = [0,[-1,-1]]  #this can be treated as visited
    
   ### This function executes when the goal node is found #####
    def backtrack(tup):  
        
        x,y = tup
        ans = []
        
        sum = oq[(x,y)][0]
        ans.append([x,y])
        
        while x!=xi or y!=yi:
            img[x][y] = 10
            x_new, y_new = oq[(x,y)][1]
            ans.append([x_new,y_new])
            x,y = x_new, y_new
        
        print("Cost to come : ", sum)
        print("Backtracking Path from goal point to start point")
        print(ans)
        # cv2.imshow("Shortest Path",img)
        # plt.imshow(img)
        # plt.show()

        cv2.imshow("Final Path", img)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
        exit()

    while Q:
        ele = hq.heappop(Q)
        #print(ele[2])
        cq[ele[2]] = 1  #adding into the closed queue should include parent as well
              
        if ele[2] == Goal_var:        #if the coordinates of current node is equal to goal node
            #backtracking, successful return
            cv2.destroyAllWindows() 
            print("Found")
            backtrack(ele[2])
            

        else:
            x,y = ele[2]  #getting the coordinate position of the element
            cost2come = ele[0]
            #print(cost2come)
            if (x,y) in oq:  #this condition can be removed
                
                if oq[(x,y)][0] < cost2come:
                    cost2come = oq[(x,y)][0]
        
            #there are 8 location for the coordinate moment
            if img[x][y] != 0:
                img[x][y] = 130
            cv2.imshow("Animation", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            for child_x,child_y,c2c in allpossiblesteps(x,y):

                if (child_x,child_y) not in cq and img[x][y] != 0:

                    if (child_x,child_y) not in oq:
                        fc2c = round(cost2come + c2c,3)
                        hq.heappush(Q,(fc2c,(x,y),(child_x,child_y))) #putting in openlist
                        hq.heapify(Q) 
                        oq[(child_x,child_y)] = [fc2c,[x,y]]
                        
                    else:
                        if oq[(child_x,child_y)][0] > cost2come + c2c:
                            oq[(child_x,child_y)][1] = [x,y]
                            oq[(child_x,child_y)][0] = round(cost2come + c2c,2)


    print("no solution found")
    
    cv2.destroyAllWindows()     

if __name__ == "__main__":
    main()
