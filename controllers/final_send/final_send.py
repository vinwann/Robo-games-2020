import mazemap
import copy
import math
#import numpy as ny
import time
import threading
from controller import Robot
from controller import RangeFinder
from controller import Keyboard
from controller import Camera
from controller import Display, ImageRef

robot = Robot()
TIME_STEP = 64


confirm_mif = [None,0]

#store vars-------------------------------------

store_para_v = [False,False] 
#-----------------------------------------------
corner_cam_on = [True]

path_started = [False]

ssp = [[]]
turn_to = [-1]

done_s1 = [False]

enterPoint = []
map_start_point = [0,-2.25]
#####################################################
enteredMaze = [False,None]
#ds_turned = [0]
#####################################################

fpt = [False]

exit_take_san = [False]


route1 = [[]]
returnRoute = [[]]
men_list = [[]]

lastTurn = [0]
steps_to_go = [100]
inJ = [False]

s2 = [0,0]

started_path = [False]

cor_pic_taken = [False]

inSignal = [False]

in_junc_cen = [False]
jTurn = [0]
cont_avoid = [False]

j_mid_reached = [False]
junc_turned = [False]

turnBaseSpeed = [3.0]
a_error = [0.002]

#obsPos = [-0.06,-1.63]
obsPos = [[]]
msg = [0]

sanPos = [[]]
last_junction = [[0,0]]

storeReached = [False]
returnJ = []

task_completed = [False]

f2 = [False,0]
taking_corner_pic = [False]
pp_middle = []

found_obs_far = [False]

#Initializing sensors/cameras ++++++++++++++++++++++++++++++++++++++++++++++++++++++

mid_block_reached = [False]
obs_in_corner = [False]
sani_incorner = [False]
obs_in_juction = [False]
man_infront = [False]
sani_infront = [False]

status = [mid_block_reached[0], obs_in_corner[0], sani_incorner[0], man_infront[0], sani_infront[0], inSignal[0]]
movedH = [False]

wall_dis = [300]
abc = []

wall_check_v = [False,None]

dis2 = robot.getDisplay("display2")

#init GPS --------------------------------------------------------------
gps = robot.getGPS("gps")
gps.enable(20)
#-----------------------------------------------------------------------



ds = []
dsNames = ['ds_left', 'ds_right', 'ds_mid']
for i in range(3):
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)

timestep = int(robot.getBasicTimeStep())

compass = robot.getCompass("compass")
compass.enable(TIME_STEP)

receiver = robot.getReceiver("receiver")
receiver.enable(20)

wheels = []
#########################################################################################
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4','servo','ds_l','ds_r','lm_r','lm_l']
for i in range(9):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
##########################################################################################    

ws = [0.0,0.0]

cam =  robot.getCamera("camera")
cam.enable(TIME_STEP)
cam_height = cam.getHeight()
cam_width = cam.getWidth()


rf = robot.getRangeFinder("range-finder")

kf = Keyboard()
kf.enable(TIME_STEP)
rf.enable(TIME_STEP)
dis = robot.getDisplay("display")
dis2 = robot.getDisplay("display2")
rf_height = rf.getHeight()
rf_width = rf.getWidth()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#All the methods ++++++++++++++++++++++++++++++++++++++++++++++++++++++++


def takePic(k=0):
    image = cam.getImageArray()
    data = get_object(image)
    print(data,len(sanPos[0]),k)
    if(len(data) > 0):
        if(data[0][2] == 'R' and len(obsPos[0]) == 0 and len(sanPos[0]) == (0 + k)):
            mazemap.storeMan(men_list[0],0,len(men_list) - 1,data[0][:2])
            del(data[0][2])
            obsPos[0] = data[0]
            obsPos_changed = False
        elif(data[0][2] == 'B' and len(sanPos[0]) == 0):
            del(data[0][2])
            sanPos[0] = data[0]
            sanPos_changed = False
            ssp[0] = copy.deepcopy(sanPos[0])

#Distance sensors turning method-----------------------------------------
def turn_ds(heading,de=False):
    angle = None
    if heading == 0 or heading == 4:
        angle = getComDir()[1]-1.570797 
       
    elif heading == 2  :
        angle = 1.570797-abs(getComDir()[1]) 
    elif heading == 3:
        angle = getComDir()[1]
    else:
        temp = getComDir()[1]
        if temp<0:
            angle = 1.570797*2 - abs(temp)
        else:
            angle =  abs(temp) - 1.570797*2
    if de :
        wheels[5].setVelocity(3.0)
        wheels[5].setPosition(0)
        wheels[6].setVelocity(3.0)   
        wheels[6].setPosition(0)                 
    elif angle != None:    
        wheels[5].setVelocity(3.0)
        wheels[5].setPosition(angle)
        wheels[6].setVelocity(3.0)   
        wheels[6].setPosition(angle)
def slide_ds(char):
    if char == "R":
        wheels[7].setVelocity(3.0)
        wheels[7].setPosition(0.1)
        robot.step(100)
    elif char == "L":
        wheels[8].setVelocity(3.0)
        wheels[8].setPosition(-0.1)
        robot.step(100)
    elif char == "B":
        wheels[7].setVelocity(3.0)
        wheels[7].setPosition(0)
        wheels[8].setVelocity(3.0)
        wheels[8].setPosition(0)
        robot.step(100)        
#------------------------------------------------------------------------


# Getting the closest distance when going towards a wall in obs avoid path
def getClosestDis():
    lt = lastTurn[0] % 4
    r_dir = getComDir()[0]
    if(lt == 0 and r_dir >= 2):
        lt = 4
    a1 = abs(lt - r_dir)*90/180*math.pi #robots angle with the last turn direction in rad
    
    d1 = round(0.06/math.tan(a1),2)
    

    d1 = d1/0.5*1000

    return d1
     


####################################Store code         

def turnAng_v(a):

    cDirec = getComDir()[0]
    start_angle = getComDir()[0]
    a2 = round((start_angle + a/90)%4,3)
    a2 = a2 % 4

    q_prev = 1
    if(a < 0):
        q_prev = -1
    s = 1.0
    cw = 0
    while(getComDir()[0] != a2):
        cDirec = getComDir()[0]

        if(a2 < cDirec):
            cw = a2 + 4 - cDirec
        else:
            cw = a2 - cDirec
        
        if(cw <= 2):
            q = 1
        else:
            q = -1

        if(q_prev != q):
            s *= 0.5
            q_prev = q
       
        leftSpeed = 3.0*q*s
        rightSpeed = -3.0*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)

def forward_v():
     wheels[0].setVelocity(3)
     wheels[1].setVelocity(3)
     wheels[2].setVelocity(3)
     wheels[3].setVelocity(3)
                             
def stop_v():
    wheels[0].setVelocity(0)
    wheels[1].setVelocity(0)
    wheels[2].setVelocity(0)
    wheels[3].setVelocity(0)
    
def heading_correction(heading):
    if (getComDir()[0] != heading):# checking if heading is wrong
        
        heading_diff = getComDir()[0] - heading#Getting the direction difference in the range 1 to 4

        if heading_diff < -2: #Fixing a special sitataion
            heading_diff = 1.9
        if 0<heading_diff or heading_diff <=2:#Taking the turn anti clockwise
            turnAng_v(-(getComDir()[0]*90 - heading*90 ))#Turning the desired angle
            
        
        elif 0>heading_diff or heading_diff >2:#Taking the turn clockwise

            turnAng_v((getComDir()[0]*90 - heading*90 -180) )  


 
def enter_maze(heading,store_para_v,entered_pos):#Code to safely enter maze by avoiding humans
 
    r_pos = gps.getValues()
    if (getComDir()[0] != heading):
        heading_correction(heading)
    #stop(ws)
    stop_v()
    #forward(ws)
    
    data = []    
    check = get_object(cam.getImageArray())#Taking pictures
    if len(check)!=0:
        for i in check:
            data.append(i)
    
    check = turn_cam(1.5708) #Turning cam and taking pictures
    if len(check)!=0:
        for i in check:
            data.append(i)    
    check = turn_cam(-1.5708) 
    if len(check)!=0:
        for i in check:
            data.append(i)
   
    wheels[4].setVelocity(3.0)
    wheels[4].setPosition(0)
  
    
    robot.step(1500)
    entered_pos[0] = r_pos[0]
    entered_pos[1]=r_pos[2]
    
    if len(data) == 0:
        
        move_to_block(heading,0.13)
        stop_v()
    else:
        n_data = []
        for i in data:
            if checking_if_infront(heading,i[:2]) and get_distance(i[:2])<0.5 and not in_list(i[0],i[1],n_data,i[3],i[2]):#Filtering objects to get what we want
                n_data.append(i)
        if len(n_data) == 0:
            move_to_block(heading,0.13)
            stop_v()
        else:
            for i in n_data:
                cord = get_cord(heading,i,0.34,0.25)#Getting cordinates to Avoid
                if cord[1] != 0 and  cord[0] != 0:
                    gottopoint(cord,heading)  #Moving robot to position                        
    cord = [0,0]                                                               
    if (ds[0].getValue()*50/1000)<11.5:#Making sure there is a safe distance between walls
        
        r_pos = gps.getValues()
        c = [r_pos[0],r_pos[2]]
        if  heading == 0 or heading == 4 :
            c[0] -= (ds[0].getValue()*50/1000)
            c[1] += 0.1
        elif heading == 2:
            c[0] += (ds[0].getValue()*50/1000)
            c[1] -= 0.1
        elif heading == 1:
            c[1] -= (ds[0].getValue()*50/1000)
            c[0] -= 0.1
        else:
            c[1] += (ds[0].getValue()*50/1000)
            c[0] += 0.1             
            
        cord =  get_cord(heading,c,0.25,0.05) 
    elif (ds[1].getValue()*50/1000)<11.5:                        
        r_pos = gps.getValues()
        c = [r_pos[0],r_pos[2]]
        if  heading == 0 or heading == 4 :
            c[0] += (ds[1].getValue()*50/1000)
            c[1] += 0.1
        elif heading == 2:
            c[0] -= (ds[1].getValue()*50/1000)
            c[1] -= 0.1
        elif heading == 1:
            c[1] += (ds[1].getValue()*50/1000)
            c[0] -= 0.1
        else:
            c[1] -= (ds[1].getValue()*50/1000)             
            c[0] += 0.1
        cord =  get_cord(heading,c,0.25,0.05) 
    if cord[1] != 0 and  cord[0] != 0:

        gottopoint(cord,heading)
    store_para_v[0] = False
    store_para_v[1] = True      


def get_cord(heading,i,x,y): #Getting a cordinate
    r_pos = gps.getValues()
    cord = [0,0]
    if heading == 0 or heading == 4 or heading == 2:
        side_ways = i[0] - r_pos[0] 
        if side_ways<0:
            cord[0] = i[0]+x
        else:
            cord[0] = i[0]-x
        if heading == 0 or heading == 4 :
            cord[1] =  r_pos[2] +y
        else:
            cord[1] =  r_pos[2] -y
    else:
        side_ways = i[1] - r_pos[2] 
        if side_ways<0:
            cord[1] = i[1]+x
        else:
            cord[1] = i[1]-x
        if heading == 3:
            cord[0] =  r_pos[0] +y
        else:
            cord[0] =  r_pos[0] -y
    return cord

def gottopoint(cord,heading):#Moving robot to that position
    r_pos = gps.getValues()############################Fixing
    bottom = 0
    top = 0
    angle = 0
    if heading == 0 or heading == 4 or heading == 2:
        top = cord[0] -  r_pos[0]
        if heading == 0 or heading == 4:
            top = -top
        bottom = abs(cord[1] -  r_pos[2])
        #angle = top/bottom
    else:
        top = cord[1] -  r_pos[2]
        
        if heading == 1:
            top = -top
           
        bottom = abs(cord[0] -  r_pos[0])
    if bottom != 0:    
        angle = top/bottom    
       
    turnAng(((math.tanh(angle))/math.pi)*180 )
    move_to_block(heading,bottom) 
    stop_v()    
    if (getComDir()[0] != heading):
        heading_correction(heading)
        stop_v()         
def get_expected_cord(heading,val):
    r_pos = gps.getValues()
    if heading == 0  or  heading == 4 or heading == 2:#Using heading to select axis
        if heading == 0  or  heading == 4:#Selection axis direction
            excepted_pos = r_pos[2] + val#getting expected possition
        elif heading == 2 :
            excepted_pos = r_pos[2] - val

           
    else:         
        if heading == 1 :
            excepted_pos = r_pos[0] - val
        elif heading == 3 :
            excepted_pos = r_pos[0] + val
    
    return excepted_pos
def move_to_block(heading,val):
    excepted_pos=get_expected_cord(heading,val) 
    if heading == 0  or  heading == 4 or heading == 2:
        while  abs(gps.getValues()[2] - excepted_pos)>0.005 :#Moving to location
            #forward(ws)
            forward_v()
            robot.step(1)     
    else:
              
        while  abs(gps.getValues()[0] - excepted_pos) >0.005 :
            #forward(ws)
            forward_v()
            robot.step(1)

def turn_cam(val=0):
   
    wheels[4].setVelocity(3.0)
    wheels[4].setPosition(val)
    
    
    robot.step(1500)
    return get_object(cam.getImageArray(),val)    

def get_distance(cord):
    r_pos = gps.getValues()
    return ((cord[0]-r_pos[0])**2+(cord[1]-r_pos[2])**2)**0.5                    
def checking_if_infront(heading,cord):
    r_pos = gps.getValues()

    if heading == 0  or  heading == 4 or heading == 2:#Using heading to select axis
        if (heading == 0  or  heading == 4) and ((r_pos[2]+0.04)<cord[1]):#Selection axis direction
            return True    
        elif heading == 2 and (r_pos[2]-0.04>cord[1]):
            return True
        else:
            return False        
   
    else:         
        if heading == 1 and (r_pos[0]-0.04>cord[0]):
            return True
        elif heading == 3 and (r_pos[0]+0.04<cord[0]):
            return True
        else:
            return False 

def store_look(data):
    for i in data:
        if i[2] == "S":
            return True
    return False        
               
def finding_store(heading,store_para_v):#Finding store

    
    store_checking_pos = [0,0]
    entered_pos = [0,0]
    found_store = False

    if  store_para_v[0] : #Entering maze safely
        enter_maze(heading[0],store_para_v,entered_pos)
    
    if store_para_v[1] : #Looking for store can find the store even if its 5m away
        r_pos = gps.getValues()
        store_checking_pos = [r_pos[0],r_pos[2]]
        data = []
        check = get_object(cam.getImageArray())
        if len(check)!=0:
            for i in check:
                data.append(i)
        if not store_look(data) :

            check = turn_cam(1.5708)
            if len(check)!=0:
                for i in check:
                    data.append(i)    
            if not store_look(data):

                check = turn_cam(-1.5708)
                if len(check)!=0:
                    for i in check:
                        data.append(i)
   
        wheels[4].setVelocity(3.0)
        wheels[4].setPosition(0)
        
        
        robot.step(1500)

        found_store = store_look(data)
        if found_store:
            print("Found store")
            return_to_entrance(heading,store_checking_pos,entered_pos)
        else:
            last_resort = []
            turnAng(90)
            stop_v()
            check = turn_cam(-1.5708)
            stop_v()
            if len(check)!=0:
                for i in check:
                    last_resort.append(i)    
            if not store_look(last_resort):
                turnAng(-180)
                stop_v()
                check = turn_cam(1.5708)
                stop_v()
                if len(check)!=0:
                    for i in check:
                        last_resort.append(i)
            turnAng(90)
            wheels[4].setVelocity(3.0)
            wheels[4].setPosition(0)
            found_store = store_look(last_resort)
            if found_store:
                print("Found store")
                return_to_entrance(heading,store_checking_pos,entered_pos)
            else:
                forward_v()
                robot.step(800)
                stop_v()
                data = []
                check = get_object(cam.getImageArray())
                if len(check)!=0:
                    for i in check:
                        data.append(i)
                if not store_look(data) :

                    check = turn_cam(1.5708)
                    if len(check)!=0:
                        for i in check:
                            data.append(i)    
                    if not store_look(data):

                        check = turn_cam(-1.5708)
                        if len(check)!=0:
                            for i in check:
                                data.append(i)
   
                wheels[4].setVelocity(3.0)
                wheels[4].setPosition(0)
                robot.step(1500)
                found_store = store_look(data)
                if found_store:
                    print("Found store")
                    return_to_entrance(heading,store_checking_pos,entered_pos)
                else:

                    return_to_entrance(heading,store_checking_pos,entered_pos)
        
                        
        store_para_v[1] = False
        storeReached[0] = True ##################storereached Kamthak danna  
        
def return_to_entrance(heading,store_checking_pos,entered_pos):

    heading[0] += 2#################Chenges last turn ** Dont change name**
    heading[0] %=4
    if (getComDir()[0] != heading[0]):
        heading_correction(heading[0])
    gottopoint(store_checking_pos,heading[0])
    gottopoint(entered_pos,heading[0])      

#getting Q---------------------------------------------------------------
def getQ(dir,rPos,pPos):
    q = 1
    if(dir%2 != 0):
        if(rPos[0] < pPos[0] and dir == 1):
            q = -1

        elif(rPos[0] > pPos[0] and dir == 3):
            q = -1

        else:
            pass

    else:
       if(rPos[0] < pPos[0] and dir == 0):

           q = -1 
       elif(rPos[0] > pPos[0] and dir == 2):
           q = -1

       else:
           pass
    return q    
#------------------------------------------------------------------------

# take a sanitizer located infront---------------------------------------
def takeSanFront():
    if(ds[0].getValue() < ds[1].getValue()):
        turn_to[0] = 1
    cord = [round(n,3) for n in gps.getValues()]
    k1 = 0
    k2 = 0
    if(round(getComDir()[0]) % 2 != 0):
        k1 = 1
        k2 = -2
        cord.reverse()
        
    start_ry = cord[2]

    if(lastTurn[0]% 4 == 0 or lastTurn[0]%4 == 3):
        lev1 = round(ssp[0][1-k1] - 0.07,2)
    else:
        lev1 = round(ssp[0][1-k1] + 0.07,2)


    if(not movedH[0]):
        l1 = [ds[0].getValue(),ds[1].getValue()]
        q = 1
        if(l1[0] < l1[1]):
            q = -1
        d1 = round(min(l1)/1000*0.5,2)
        d2 = d1 - 0.1

        if(min(l1) > 250):
            moveH(q,d2,0.04)
            
            if(lastTurn[0]%4 == 0 or lastTurn[0]%4 == 3):
                while(round(gps.getValues()[2+k2],2) < lev1):
                    robot.step(10)

                    wheels[0].setVelocity(3.0)
                    wheels[1].setVelocity(3.0)
                    wheels[2].setVelocity(3.0)
                    wheels[3].setVelocity(3.0)
                done_s1[0] = True
            else:
                while(round(gps.getValues()[2+k2],2) > lev1):
                    robot.step(10)
                    wheels[0].setVelocity(3.0)
                    wheels[1].setVelocity(3.0)
                    wheels[2].setVelocity(3.0)
                    wheels[3].setVelocity(3.0)
                done_s1[0] = True
            
        else:
            movedH[0] = True
            
    else:      
        if(lastTurn[0]%4 == 0 or lastTurn[0]%4 == 3):
            while(round(gps.getValues()[2+k2],2) < lev1):
                robot.step(10)

                wheels[0].setVelocity(3.0)
                wheels[1].setVelocity(3.0)
                wheels[2].setVelocity(3.0)
                wheels[3].setVelocity(3.0)
            done_s1[0] = True
        else:
            while(round(gps.getValues()[2+k2],2) > lev1):
                robot.step(10)
                wheels[0].setVelocity(3.0)
                wheels[1].setVelocity(3.0)
                wheels[2].setVelocity(3.0)
                wheels[3].setVelocity(3.0)
            done_s1[0] = True
#------------------------------------------------------------------------

#get pic while following a path------------------------------------------
#this returns path points if it finds a man infront of it.(to execute 3rd point to 2nd point move)
def getPathPic(q = 0):

    image = cam.getImageArray()
    data = get_object(image,q * 1.570797)
    
    rPos = [gps.getValues()[x] for x in range(0,3,2)]
    n = 0
    dir =lastTurn[0]
    if(dir%2 == 0):
        n = 1
 
    if(len(data) > 0):
        if(data[0][2] == 'R'):
            mazemap.storeMan(men_list[0],0,len(men_list) - 1,data[0][:2])
            del(data[0][2])
            d_backup = data[0]
            obsPos[0] = data[0]
            
            dis_to_sm = round(abs(rPos[n]-obsPos[0][n]),2)
            limit = 0.4
            dis_to_front = round(ds[2].getValue()/1000*0.5,2)
            man_to_wall_dis = round(0.1 + dis_to_front + dis_to_sm,2)

            distance_x = abs(rPos[1-n] - obsPos[0][1-n])

            if(taking_corner_pic[0]):
                c = False
                if(dir % 4 == 0 or dir%4 == 3):
                    if(((rPos[n] - obsPos[0][n]) > 0.05) and not inSignal[0]):

                        c = True
                else:
                    if(((rPos[n] - obsPos[0][n]) < -0.05) and not inSignal[0]):
                        c = True
       
                if(distance_x > 0.4):
                    found_obs_far[0] = True
                        
                if(c):
                    obsPos[0] = []

            lastD = lastTurn[0] % 4
            curD = getComDir()[0]
            if(curD > 2 and lastD == 0 ):
                lastD = 4
            angle_1 = abs(lastD - curD)*90/180*math.pi
            ds_vals = [ds[0].getValue(),ds[1].getValue()]

            width1 = round(sum(ds_vals)/1000*0.5*math.cos(angle_1) + 0.1*math.cos(angle_1),2)

            
            if(dis_to_sm <= 0.5 and (width1 > 0.6) and inSignal[0]):
                f2[0] = True
                if(len(obsPos[0]) > 0):
                    pp = getPathPoints(obsPos[:2])
                    return pp
                   
            if(dis_to_sm <= 0.5 and (width1 > 0.6) and man_to_wall_dis < 0.4):
                f2[0] = True
                if(len(obsPos[0]) > 0):
                    pp = getPathPoints(obsPos[:2])
                    return pp
                    
            
                    
            elif(dis_to_sm <= 0.5 and (width1 < 0.55)):
                f2[0] = True
                if(len(obsPos[0]) > 0):
                    pp = getPathPoints(obsPos[:2])
                    return pp
                    
        elif(data[0][2] == 'B'):
            f2[0] = False
            del(data[0][2])
            sanPos[0] = data[0]

    
#------------------------------------------------------------------------

#Go to center------------------------------------------------------------
#calculates the distance to move to the centre and moves to the relv. direc
def gotoCen():

    l1 = [ds[0].getValue(),ds[1].getValue()]
    ds_val = round(min(l1),2)

    d1 = round(ds_val/1000*0.5,2) + 0.06
    d2 = round(0.25 - d1,2)

    q = 1
    if(l1[0] > l1[1]):
        q = -1

    moveH(q,q*d2, 0.05)
    stop(ws)
    robot.step(10)
    takePic()
    
    
#------------------------------------------------------------------------
#Horizontal movements----------------------------------------------------
#Moves x length horzontally going y length forward
# Should include the direction to rotate 
# rotating directions == | clockwise ==> [q = +1] | counter clock ==> [q = -1]
def moveH(q,x = 0.1,y = 0.04):
    dir = round(getComDir()[0])
    rPos = [gps.getValues()[x] for x in range(0,3,2)]
    if(dir%2 != 0):
        rPos.reverse()

    k = 0
    if(dir == 1 or dir == 2):
        s = -1
    else:
        s = 1
    if(dir%2 != 0):
        k = -2

    ang1 = math.atan(abs((x/y)))/math.pi*180

    turnAng(ang1*q)
    while(round(gps.getValues()[2+k],2) != round(rPos[1] + y*s,2)):
        robot.step(10)
        wheels[0].setVelocity(3.0)
        wheels[1].setVelocity(3.0)
        wheels[2].setVelocity(3.0)
        wheels[3].setVelocity(3.0)
    turnAng(-ang1*q)
    movedH[0] = True

              
#------------------------------------------------------------------------

#Corner obstacle avoidance-----------------------------------------------
#Moves the robot further to the wall to avoid a human located in a turn , infront of it 
#|      !robot
#|     \|/ 
#|
#|human*
#|_____________wall
def cObsAvoid():
    d = 1
    if(man_infront[0]): d = 0
    (confirm_mif[0],confirm_mif[1]) = (None,0)
    k = 0
    cord = [round(n,3) for n in gps.getValues()]
    if(round(getComDir()[0])%2 != 0):
        k = -2
        cord.reverse()
        
    if(round(getComDir()[0])%2 == 0):
        obsY = obsPos[0][1]
        obsX = obsPos[0][0]
    else:
        obsY = obsPos[0][0]
        obsX = obsPos[0][1]
        
    if(lastTurn[0]%4 == 0 or lastTurn[0] %4 == 3):
        target1 = round(obsY + 0.3,2)
    else:
        target1 = round(obsY - 0.3,2)
    
    q = 1    
    if(lastTurn[0]%4 == 0 or lastTurn[0] %4 == 1):
        if(obsX > cord[0]):
            q = -1
    else:
        if(obsX < cord[0]):
            q = -1
    
    while(round(gps.getValues()[k+2],2) != target1):
        if(ds[2].getValue() <= 900):
            return
        robot.step(10)
        wheels[0].setVelocity(3.0)
        wheels[1].setVelocity(3.0)
        wheels[2].setVelocity(3.0)
        wheels[3].setVelocity(3.0)

    turnAng(q*90)
    
    cord = [round(n,3) for n in gps.getValues()]
    if(round(getComDir()[0])%2 != 0):
        cord.reverse()
    
    if(abs(cord[2] - round(obsX + 0.13,2)) < abs(cord[2] - round(obsX - 0.13,2))):
        target2 = obsX + 0.13
    else:
        target2 = obsX - 0.13
    target2 = round(target2,2)    

    while(round(gps.getValues()[-k],2) != target2):
        robot.step(10)
        wheels[0].setVelocity(3.0)
        wheels[1].setVelocity(3.0)
        wheels[2].setVelocity(3.0)
        wheels[3].setVelocity(3.0)
    turnAng(-q*90)
            
#------------------------------------------------------------------------

#choose j direc----------------------------------------------------------
def getJDirec():
    msg[0][-1] = msg[0][-1][:len(msg[0][-1])-1]
    sList = []
    for steps in msg[0]:
        if(steps != '0'):
            sList.append(int(steps))
    chosen_steps = 0
    
    chosen_steps = str(max(sList))
    stop(ws)

    steps_to_go[0] = int(chosen_steps)
    turnDirec = msg[0].index(chosen_steps)
    currentDirec = lastTurn[0]

    if(storeReached[0]):
        turnDirec = returnJ[-1]
        returnJ.pop()
    else:
        returnJ.append((currentDirec-2)%4)

    if(turnDirec != currentDirec and not cont_avoid[0]):

        jTurn[0] = turnDirec
        in_junc_cen[0] = True
    else:
        cont_avoid[0] = True
                
    j_mid_reached[0] = True

#------------------------------------------------------------------------

#Comms method -----------------------------------------------------------
def comms():
    
    cur_strength = 0
    prev_strength = 0
    while(True):
        
        if(task_completed[0]):
            return
        if(receiver.getQueueLength() > 0):

            inSignal[0] = True

            msg[0] = receiver.getData().decode('utf-8').split(",")

            cur_strength = round(receiver.getSignalStrength(),3)

            if(cur_strength < 8.4):
                junc_turned[0] = False
                prev_strength = 0
                cont_avoid[0] = False
                j_mid_reached[0] = False
                cor_pic_taken[0] = False
                inSignal[0] = False

            receiver.nextPacket()
        else:
            pass
            
        
 
        if(cur_strength < prev_strength and cur_strength > 8 and not junc_turned[0] and not j_mid_reached[0]):
            getJDirec()
        
           
        if(man_infront[0] and inSignal[0] and not junc_turned[0] and not j_mid_reached[0]):  
            getJDirec()   
    
        
        prev_strength = cur_strength
#------------------------------------------------------------------------
#taking a sanitizer------------------------------------------------------
#V shaped path to take a sanitizer---------------------------------------
def takeSan(spp):

    dir = round(getComDir()[0]) % 4
    rPos = [gps.getValues()[x] for x in range(0,3,2)]
    sPos = sanPos[0][0]
    if(dir%2 != 0):
        sPos = sanPos[0][1]
        rPos.reverse()

    q = 1
    k = 0
    if(dir%2 != 0):
        k = -2
        if(rPos[0] < sPos and dir == 1):
            q = -1

        elif(rPos[0] > sPos and dir == 3):
            q = -1

        else:
            pass

    else:
       if(rPos[0] < sPos and dir == 0):
           q = -1 
       elif(rPos[0] > sPos and dir == 2):
           q = -1
       else:
           pass

    ang1 = math.atan(abs((rPos[0]-sPos)/0.3))/math.pi*180
    turnAng(ang1*q)
    for n in range(1,5):
        if(n%2 == 0):
            if(n%4 == 0):
                turnAng(ang1*q)
                getPathPic()
                f2[0] = False

            else:
                turnAng(-2*ang1*q)
                takePic(3)
                if(len(obsPos[0]) > 0):
                    return
                
            
        else:
            n2 = 5 % n
            if(n2 == 0): n2 = 1
            while(round(gps.getValues()[2+k],2) != round(spp[n2],2)):
                if(j_mid_reached[0]):
                    if(jTurn[0] != dir):
                        exit_take_san[0] = True

                if(n==3 and exit_take_san[0]):
                    return
                if(n == 3 and ds[2].getValue() < 200):
                    align()
                    return
                    
                robot.step(10)
                wheels[0].setVelocity(3.0)
                wheels[1].setVelocity(3.0)
                wheels[2].setVelocity(3.0)
                wheels[3].setVelocity(3.0)
#------------------------------------------------------------------------
#obstacle avoidance using GPS -------------------------------------------
def gObsAvoid(pp):
    (confirm_mif[0],confirm_mif[1]) = (None,0)
    qSecond = 0

    dir  = lastTurn[0] % 4

    rPos = [gps.getValues()[x] for x in range(0,3,2)]
    oPos = obsPos[0][:2]
    if(dir%2 != 0):
        oPos.reverse()
        rPos.reverse()
    pPos = pp[1]

    k = 0

    if(dir%2 != 0):
        k = -2
    q = getQ(dir,rPos,pPos)  


    if(q == 1):
        d2 = round(ds[0].getValue()/1000*0.5,2) + 0.05
    else:
        d2 = round(ds[1].getValue()/1000*0.5,2) + 0.05
        
    rx = round(rPos[0],2)
    obsx = oPos[0]
    d1 = abs(rx - obsx) #distance between the robot and the obs in x direction
    d3 = round(d2 - d1,2) #distance between the obs and the wall    
    redAng = 0
    if(d3 >= 0.1):
        redAng = 3
        

    ang1 = ( math.atan(abs((rPos[0]-pPos[0])/(rPos[1]-pPos[1])))/math.pi*180 + 2 ) - redAng
    ang1r = math.atan(abs((rPos[0]-pPos[0])/(rPos[1]-pPos[1])))

    d1 = round(abs(rPos[0]-pPos[0]),2)
    d2 = round(math.cos(ang1r)*0.05 + math.sin(ang1r)*0.1,2)

    lt = lastTurn[0]%4
    cd = getComDir()[0]
    if(lt == 0 and cd > 3):
        lt = 4
    
    con1 = abs(lt - cd) > 0.02
    move_out = False
    if(con1):
        ang2 = round(abs(cd - lt)*90)
        q2 = 1
        if(lt % 4 == 0 and cd > 3):
            q2 = -1
        elif(cd < lt%4):
            q2 = -1
        a3 = round(q*ang1 - q2*ang2)
        ang1 = a3
        qSecond = -(ang1/abs(ang1))
        turnAng(ang1)
        
    else:
        dif = round(pp[0][1] - rPos[1],2)

        should_move_out = False
        if(dir == 0 or dir == 3):
            if(dif < 0):
                should_move_out = True
        else:
            if(dif > 0):
                should_move_out = True
        if(should_move_out):
            ang1 = 90
            move_out = True
        qSecond = -(ang1/abs(ang1))
        turnAng(ang1*q)
        
    angles = []
    for n in range(1,7):
        if(f2[0]):
            f2[0] = False
            gObsAvoid(pp2)
            return
        if(n%2 == 0):
            if(n == 4):
                wall_check_v[0] = True
                pp2 = getPathPic()
                if(wall_check_v[1] != None):
                    (wall_check_v[0],wall_check_v[1]) = (False,None)
                    return
                (wall_check_v[0],wall_check_v[1]) = (False,None)
                
                if(f2[0]):
                    f2[1] = -q
                else:
                    pPos = pp[3]
                    rPos = pp[2]
                    q = getQ(dir,rPos,pPos)
                    turnAng(50*q)

                    
                    pp2 = getPathPic()
                    if(f2[0]):
                        f2[1] = q
                
            elif(n == 6):
                align()
                started_path[0] = False

            else:
                align()
                f2[0] = False
                
        else:
            n2 = 8 % n
            if(n2 == 0): n2 = 1
            t = 0
            startV = 0
            rate = 0
            error = 0.04
            if(round(gps.getValues()[2+k],2) > round(pp[n2][1],2)):
                error *= -1
            d1 = 0
            while((round(gps.getValues()[2+k],2) != round(pp[n2][1],2) )and ( round(gps.getValues()[2+k],2) != round(pp[n2][1],2) + error) ):
                if((n == 1 or n == 5) and d1 == 0):
                    d1 = getClosestDis()
                if(n == 1):
                    started_path[0] = True
                if(n == 1 and move_out):
                    while(ds[2].getValue() >= 60):
                        robot.step(10)
                        wheels[0].setVelocity(3.0)
                        wheels[1].setVelocity(3.0)
                        wheels[2].setVelocity(3.0)
                        wheels[3].setVelocity(3.0)
                    turnAng(-90*q)
                    
                    move_out = False
                    break
                    
                if(n == 3): t += 1
                if(n == 3 and (t == 10 or t == 100)):
                    a_dis = 0
                    image = rf.getRangeImage()
                    for i in range(rf_width//5,rf_width//5*4):
                        for j in range(rf_height//3,rf_height//3*2):
                            a_dis += RangeFinder.rangeImageGetDepth(image,rf_width,i,j)
                    if(t == 10): startV = a_dis
                    if(t == 100): rate = (startV - a_dis)/90
                    
                    if(rate >= 5):
                        started_path[0] = False
                        return
               
                if(in_junc_cen[0]):
                    started_path[0] = False
                    return
                if(mid_block_reached[0]):
                    return
                
                if(ds[2].getValue() <= d1 + 40):
                    break   
                robot.step(10)
                wheels[0].setVelocity(3.0)
                wheels[1].setVelocity(3.0)
                wheels[2].setVelocity(3.0)
                wheels[3].setVelocity(3.0)
                
    
    
#-----------------------------------------------------------------------

#get sanitizer path points----------------------------------------------
def getSPoints(sanPos):
    dir = round(getComDir()[0])%4

    yPos = sanPos[0][1]
    s = 1
    if(dir%2 != 0):
        yPos = sanPos[0][0]
        s = -1
    rPos = [round(gps.getValues()[x],3) for x in range(0,3,2)]
    if(s == -1):
        rPos.reverse()

    sppList = [round(yPos - 0.2,2), yPos , round(yPos + 0.2,2)]
    if(dir == 2 or dir == 1):
        sppList.reverse()

    return sppList


#----------------------------------------------------------------------


#calculate and return path points---------------------------------------
def getPathPoints(obsPos):
    dir = lastTurn[0]
    xPos = obsPos[0][0]
    yPos = obsPos[0][1]
    s = 1
    rPos = [round(gps.getValues()[x],3) for x in range(0,3,2)]
    if(dir%2 != 0):
        s = -1
        xPos = obsPos[0][1]
        yPos = obsPos[0][0]
        rPos.reverse()
     
    q = 1
    u = 1
    if(dir%2 == 0): 
        if(xPos < rPos[0]):
            q = -1
        else:
            pass
    else:
        if(xPos > rPos[0]):
            q = -1
        else:
            pass
        
    
    ppX = [0]*4
    ppY = [0]*4
    for x in range(4):
        if(x%3 == 0):
            ppX[x] = xPos -0.182*q*s
        else:
            ppX[x] = xPos -0.282*q*s
    for y in range(4):
        a = 1
        if(y%3 == 0):
            if(y == 0):
                a = -1
            ppY[y] = yPos + 0.294*a
        else:
            if(y == 1):
                a = -1
            ppY[y] = yPos + 0.214*a 

    ppList = [(round(ppX[n],3),round(ppY[n],3)) for n in range(4)]
    if(dir == 2 or dir == 1):
        ppList.reverse()

    return ppList         
#-----------------------------------------------------------------------


#Pause method----------------------------------------------------------
def pause(t):
    t *= 100
    while(t > 0):
        robot.step(10)
        t -= 1
#-----------------------------------------------------------------------

#Clear buffer-----------------------------------------------------------
def clearBuffer():
    while(receiver.getQueueLength() > 0):
        receiver.nextPacket()
#-----------------------------------------------------------------------

#Capture corner image--------------------------------------------------------
def cornerPic(q = 0):
    ssp[0] = copy.deepcopy(sanPos[0])
    sanPos[0] = []
    taking_corner_pic[0] = True
    if(q == 0):
        if(ds[0].getValue() < ds[1].getValue()):
            q = -1
        else:
            q = 1
    stop(ws)
    wheels[4].setVelocity(3.0)
    wheels[4].setPosition(q*1.57)
    robot.step(1500)
   
    pause(0.2)
    
    getPathPic(q)
    if(f2[0]):
        obs_in_corner[0] = True
        f2[0] = False
        
    if(len(sanPos[0]) > 0):
        pass
        
    wheels[4].setVelocity(3.0)
    wheels[4].setPosition(0)
    robot.step(1500)

    cor_pic_taken[0] = True
    if(len(sanPos[0]) > 0):
        sani_incorner[0] = True

    taking_corner_pic[0] = False
#----------------------------------------------------------------------------

#Aligning the robot ---------------------------------------------------------        
def align():
    if(lastTurn[0] == 1 ):
        turnEast()
    elif(lastTurn[0] == 2):
        turnSouth()
    elif(lastTurn[0] == 3):
        turnWest()
    else:
        turnNorth()

    
#----------------------------------------------------------------------------

#Turning methods ------------------------------------------
def turnEast():
    s = 1.0
    q_prev = 1
    cDirec = getComDir()[0]
    if(not(cDirec >= 3 or cDirec < 1)):
        q_prev = -1
    while (round(getComDir()[0],2) != 1.0 ):
        cDirec = getComDir()[0]
        if(cDirec >= 3 or cDirec < 1 + a_error[0]):
            q = 1
        else:
            q = -1
        if(q_prev != q):
            s *= 0.5
            q_prev = q
        leftSpeed = turnBaseSpeed[0]*q*s
        rightSpeed = -turnBaseSpeed[0]*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
        
    stop_v()
    turn_ds(lastTurn[0],True)
    slide_ds("B")
    robot.step(500)
def turnWest():
    s = 1.0
    q_prev = 1
    cDirec = getComDir()[0]
    if(not(cDirec >= 1 and cDirec < 3)):
        q_prev = -1
    while (round(getComDir()[0],2) != 3.0):
        cDirec = getComDir()[0]
        if(cDirec >= 1 + a_error[0] and cDirec < 3 + a_error[0]):
            q = 1
        else:
            q = -1
        if(q_prev != q):
            s *= 0.5
            q_prev = q
        leftSpeed = turnBaseSpeed[0]*q*s
        rightSpeed = -turnBaseSpeed[0]*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)

    stop_v()
    turn_ds(lastTurn[0],True)
    slide_ds("B")
    robot.step(500)

def turnSouth():
    s = 1.0
    q_prev = 1
    cDirec = getComDir()[0]
    if(not(cDirec >= 0 and cDirec < 2)):
        q_prev = -1
    while (round(getComDir()[0],2) != 2.0):
        cDirec = getComDir()[0]
        if(cDirec >= 0 and cDirec < 2 + a_error[0]):
            q = 1
        else:
            q = -1
        if(q_prev != q):
            s *= 0.5
            q_prev = q
        leftSpeed = turnBaseSpeed[0]*q*s
        rightSpeed = -turnBaseSpeed[0]*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
        
    stop_v()
    turn_ds(lastTurn[0],True)
    slide_ds("B")
    robot.step(500)

def turnNorth():
    s = 1.0
    q_prev = 1
    cDirec = getComDir()[0]
    if(not(cDirec >= 2)):
        q_prev = -1
    while (getComDir()[0] != 0.0 and getComDir()[0] != 4.0):
        cDirec = getComDir()[0]
        if(cDirec >= 2):
            q = 1
        else:
            q = -1
        if(q_prev != q):
            s *= 0.5
            q_prev = q
        leftSpeed = turnBaseSpeed[0]*q*s
        rightSpeed = -turnBaseSpeed[0]*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
   
    stop_v()
    turn_ds(lastTurn[0],True)
    slide_ds("B")
    robot.step(500)

#-----------------------------------------------------------------

#get compass direction method-------------------------------------
def getComDir():
    cVals = compass.getValues()
    xrVal = round(cVals[0],3)
    zrVal = round(cVals[2],3)
    
    rad = math.atan2(xrVal,zrVal)
    bearing = (rad - 1.5708)/math.pi * 180
    if(bearing < 0.0):
        bearing += 360
        
    
    bearing = bearing % 360
    bearing = round(bearing,3)
    comDirc = round(bearing / 90,3)

    return comDirc % 4,rad
    
#----------------------------------------------------------------

#Turning in junction - juncTurn method---------------------------

def juncTurn(direction):

    if(direction == 0 or direction == 4):
        turnNorth()
    elif(direction == 1):
        turnEast()
    elif(direction == 2):
        turnSouth()
    else:
        turnWest()

    lastTurn[0] = direction % 4
    junc_turned[0] = True
    in_junc_cen[0] = False
    man_infront[0] = False
    forward(ws)

#---------------------------------------------------------------

#Avoiding wall - obsTurn method---------------------------------
def obsTurn():
    clearBuffer()
    turnD = round(getComDir()[0])
    if(sani_infront[0]):
        turnD += turn_to[0]
    elif(ds[0].getValue() < ds[1].getValue()):
        turnD += 1
    else:
        turnD -= 1


    turnD = turnD % 4
    lastTurn[0] = turnD
   
    if(turnD == 0):
        turnNorth()
    elif(turnD == 1):
        turnEast()
    elif(turnD == 2):
        turnSouth()
    else:
        turnWest()
   
    if(man_infront[0]):
        
        cObsAvoid()
        obs_in_corner[0] = False

        obsPos[0] = []
    elif(sani_infront[0]):
        cord = [round(n,3) for n in gps.getValues()]
        k = 0
        if(round(getComDir()[0]) % 2 != 0):
            k = -2
            cord.reverse()
        start_ry = cord[2]

        while(abs(round(gps.getValues()[2+k],2) - start_ry) <= 0.35):
            robot.step(10)
            wheels[0].setVelocity(3.0)
            wheels[1].setVelocity(3.0)
            wheels[2].setVelocity(3.0)
            wheels[3].setVelocity(3.0)
        turn_to[0] = -1
    if(not sani_incorner[0]):
        sanPos[0] = []
        
    
#---------------------------------------------------------------

#Turning Angle method-------------------------------------------
def turnAng(a):
    cDirec = getComDir()[0]
    start_angle = getComDir()[0]
    a2 = round((start_angle + a/90)%4,3)
    a2 = a2 % 4
    q_prev = 1
    if(a < 0):
        q_prev = -1
    s = 1.0
    cw = 0
    while(getComDir()[0] != a2):
        cDirec = getComDir()[0]
        if(a2 < cDirec):
            cw = a2 + 4 - cDirec
        else:
            cw = a2 - cDirec
        
        if(cw <= 2):
            q = 1
        else:
            q = -1

        if(q_prev != q):
            s *= 0.5
            q_prev = q
       
        leftSpeed = 3.0*q*s
        rightSpeed = -3.0*q*s
        robot.step(1)
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)
    stop_v()
    if(not enteredMaze[0]):
        turn_ds(lastTurn[0])
        if a>0: 
            slide_ds("R")
        else:
            slide_ds("L")    
        robot.step(500)

#------------------------------------------------------------

#Image Recognition$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

############################################################################################copy

##########################################################################################

def zooming(data):#Selecting the area a object was found adding more pixels to to the currently found pixel list for further processing
    
    khaliod_pix = khaliod(data)#Getting the pixel thats closest to the center of the pixel distribution
    if khaliod_pix[0] == None:
        return None

    distance_f1 = get_object_distance([khaliod_pix[0]])*math.cos((abs(khaliod_pix[0][1]-72)/72)*0.785398) #Getting the horizontal distance to object and its center pixel
    if distance_f1<0.1 or distance_f1>0.6:
        return None 
    data_v = sorted(data,key=lambda x:x[1])#Sorting the pixel array in order of second cordinate 
    h_lower_b = data[0][0]-6#Getting boundries
    h_higher_b = data[-1][0]+6 
    v_lower_b = data_v[0][1]-8
    v_higher_b = data_v[-1][1]+8
    if h_lower_b<0:#Correcting boundries
        h_lower_b = 0
    if h_higher_b > 144:
        h_higher_b = 144
    if v_lower_b<0:
        v_lower_b = 0
    if v_higher_b > 144:
        v_higher_b = 144    
           
    lis = []
    for x in range(h_lower_b,h_higher_b):
        for y in range(v_lower_b,v_higher_b):
            lis.append([x,y]) #Getting zoomed area

    return distance_to_every_pixel(lis,distance_f1,khaliod_pix[0]) #Filtering pixels out of distance range such as sky           

def khalixis(list1): #Getting axis of the object
    if len(list1) == 0 :
        return None
    Y=sum([lis[1] for lis in list1])/len(list1)

    list2=[abs(lis[1]-Y) for lis in list1]

    m=min(list2)
    list3=[list1[x] for x in range(len(list2)) if m==list2[x] ]
    return list3[0][1]

def khaliod(list1): #Getting the pixel closest to the center
    if len(list1) == 0:
        return None
    X=sum([lis[0] for lis in list1])/len(list1)
    Y=sum([lis[1] for lis in list1])/len(list1)
    list2=[abs(lis[0]-X)+abs(lis[1]-Y) for lis in list1]
    m=min(list2)
    list3=[list1[x] for x in range(len(list2)) if m==list2[x] ]
    return list3
     
def khali_clustering(list1,num,type): #Clustering pixels
    list2=[]
    while len(list1)>0:
        ini1=list1.pop(0)
        a=[ini1]
        i=0
        while i<len(a):
            for x in range(i,len(a)):
                i+=1
                ini=a[x]
                lis=list1.copy()
                for sec in list1:
                    if ((ini[0]-sec[0])**2 + (ini[1]-sec[1])**2)**0.5 <= num*(2**0.5): #Num gives the maximum torelatable distance between two pixels
                        a.append(sec)
                        lis.remove(sec)
                else:
                    list1=lis
        else:
            list2.append(a)
    num = len(list2)
    count = 0
    while count<num:
    
        if (len(list2[count])<=300 and type == "W") or (len(list2[count])<=100 and type == "B") or (len(list2[count])<=20 and type == "R") :#Removing clusters under 20 elements
            list2.remove(list2[count])
            num -=1
            """
            if not green:
                list2.remove(list2[count])
                num -=1
            elif len(list2[count])<4:
                list2.remove(list2[count])
                num -=1 
                """   
        else:
            list2[count] = sorted(list2[count])
            count+=1
         
    return list2 

def colour_change(data,count = 0):#Function to change the colours in the picture
    if (data[0]> data[1]<95) and (data[0]> data[2]<95) :#Getting red pixels that have a heigher red value than other colours and the other colours have to have a value less than 95, The threshold values were chosen by trail and error
        if count == 0:#0.7
            num = ((data[0]-data[1]) + (data[0]-data[2]))//0.8#Adding colour value differnces and dividing it by it by a certain value . The value was choosen by trail and error
        elif count == 1:#1.4
            num = ((data[0]-data[1]) + (data[0]-data[2]))//6
        if data[0] == 255  :
            pass
        else:
            if data[0]+num <= 255:
                data[0]+=num
            else:
                data[0] = 255
          
    elif ((data[2]> data[1])and(data[1] <255)) and ((data[2]> data[0]) and (data[0]<255)) :#Getting blue pixels that have a heigher blue value than other colours and the other colours have to have a value less than 255
        if count == 0:
            num = ((data[2]-data[1]) + (data[2]-data[0]))//0.4
        elif count == 1:
            num = ((data[2]-data[1]) + (data[2]-data[0]))//2.5
        if data[2] == 255  :
            pass
        else:
            if data[2]+num <= 255:
                data[2]+=num
            else:
                data[2] = 255
    elif ((data[1]> data[2])and(data[2] <80)) and ((data[1]> data[0]) and (data[0]<80)) :#Doing the above for green
        if count == 0:
            num = ((data[1]-data[2]) + (data[1]-data[0]))//0.4
        elif count == 1:
            num = ((data[1]-data[2]) + (data[1]-data[0]))//2.5
        if data[2] == 255  :
            pass
        else:
            if data[1]+num <= 255:
                data[1]+=num

            else:
                data[1] = 255                
       
def image_process(img,width,l_height,h_height):#Image processing function
     for x in range(0,width):
        for y in range(l_height,h_height):
            colour_change(img[x][y])# Sending pixel for colour change
        
     return img       

def get_object_distance(data):#Getting object distance
    if len(data) == 0:
        return None
    rf_image = rf.getRangeImage()#Getting range finder image
    dis = 0
    for i in data:
        dis += RangeFinder.rangeImageGetDepth(rf_image,rf_width,i[0],i[1])
    return dis/len(data) #Getting avg distance to object
    
    
def distance_to_every_pixel(data,distance,c_pix):#Getting pixel distancees and removing pixels far from the object .Very important for removing sky from caught pixels,Distance is the distance from the robot to the center pixel
    if len(data) == 0:
        return None
    dis_list = []
    rf_image = rf.getRangeImage()

    for i in data:
        angle_2 = (abs(i[0]-c_pix[1])/144)*1.5708#Getting angel between center pixel and chosen pixel
        angle_1 = (abs(i[1]-72)/72)*0.785398 #Getting vertical angle for pixel
        length = RangeFinder.rangeImageGetDepth(rf_image,rf_width,i[0],i[1])*math.cos(angle_1)*math.cos(angle_2)

        if abs(length - distance) <= 0.3: #Getting piexles in range
            dis_list.append(i)
    
    return dis_list

def getting_loc(data,angle =0 ): #Getting location of a object
    robot_ori = getComDir()[1] #Getting robot orientaion in radian
    robot_pos = gps.getValues()
    location = [0,0]

    if -1.5707963 <robot_ori and robot_ori<=0:

        if  angle <0: #When camera is 90 degrees turned clockwise

            location[0] += 0.1*math.cos(robot_ori) - (data[0])*math.sin(data[1]+robot_ori) + robot_pos[0] - 0.01*math.sin(robot_ori)
            location[1] += (-0.1*math.sin(robot_ori) + (data[0])*math.cos(-data[1]+robot_ori) + robot_pos[2] + 0.01*math.cos(robot_ori))
        elif  angle >0: #When camera is 90 degress turned  anti clockwise
            location[0] += 0.1*math.cos(robot_ori) + (data[0])*math.sin(data[1]+robot_ori) + robot_pos[0] + 0.01*math.sin(robot_ori)
            location[1] += 0.1*math.sin(robot_ori) - (data[0])*math.cos(-data[1]+robot_ori) + robot_pos[2] - 0.01*math.cos(robot_ori)
        else:  #When camera is straight  
            location[0] += (0.1+0.01)*math.cos(robot_ori)+(data[0])*math.cos(data[1]+robot_ori)+robot_pos[0] 
            location[1] +=  (data[0])*math.sin(data[1]+robot_ori) +robot_pos[2] + (0.1+0.01)*math.sin(robot_ori) 
    elif 0<robot_ori and robot_ori <= 1.570797:

        if  angle <0:

            location[0] += 0.1*math.cos(robot_ori) - (data[0])*math.sin(data[1]+robot_ori) + robot_pos[0] - 0.01*math.sin(robot_ori)
            location[1] += (0.1*math.sin(robot_ori) + (data[0])*math.cos(data[1]+robot_ori) + robot_pos[2] + 0.01*math.cos(robot_ori))
        elif  angle >0:
            location[0] += 0.1*math.cos(robot_ori) + (data[0])*math.sin(data[1]+robot_ori) + robot_pos[0] + 0.01*math.sin(robot_ori)
            location[1] += 0.1*math.sin(robot_ori) - (data[0])*math.cos(data[1]+robot_ori) + robot_pos[2] - 0.01*math.cos(robot_ori)
        else:
            location[1] = (0.1+0.01)*math.cos(1.570797-robot_ori)+(data[0])*math.cos(1.570797-(data[1]+robot_ori))+robot_pos[2]
            location[0] =  (data[0])*math.sin(1.570797-(data[1]+robot_ori)) +robot_pos[0] + (0.1+0.01)*math.sin(1.570797-robot_ori)
    elif 1.570797<robot_ori and robot_ori<=3.141593:

        if  angle <0:
            location[0] += -0.1*math.cos(3.141593 -robot_ori) - (data[0])*math.sin(3.141593 -(data[1]+robot_ori)) + robot_pos[0] - 0.01*math.sin(3.141593 -robot_ori)
            location[1] += (-0.1*math.sin(3.141593 -robot_ori) - (data[0])*math.cos(3.141593 -(data[1]+robot_ori)) + robot_pos[2] - 0.01*math.cos(3.141593 -robot_ori))
        elif  angle >0:
            
            location[0] += -0.1*math.cos(3.141593 -robot_ori) + (data[0])*math.sin(-data[1] + 3.141593 -robot_ori) + robot_pos[0] + 0.01*math.sin(3.141595 -robot_ori)
            location[1] += 0.1*math.sin(3.141593 -robot_ori) + (data[0])*math.cos(data[1]+3.141593 -robot_ori) + robot_pos[2] + 0.01*math.cos(3.141595 -robot_ori)
        else:    
            location[0] = -(0.1+0.01)*math.cos(3.141593 -robot_ori)-(data[0])*math.cos(3.141593-(data[1]+robot_ori))+robot_pos[0]
            location[1] =  (data[0])*math.sin(3.141593-(data[1]+robot_ori)) +robot_pos[2] + (0.1+0.01)*math.sin(3.141593 -robot_ori)
                
    elif -3.141593 <robot_ori  and robot_ori<= -1.5707963:
        if  angle <0:

            location[0] += -0.1*math.cos(3.141593 +robot_ori) + (data[0])*math.sin(3.141593 +data[1]+robot_ori) + robot_pos[0] + 0.01*math.sin(3.141593 +robot_ori)
            location[1] += (-0.1*math.sin(3.141593 +robot_ori) - (data[0])*math.cos(3.141593 +data[1]+robot_ori) + robot_pos[2] - 0.01*math.cos(3.141593 +robot_ori))
        elif  angle >0:
            location[0] += -0.1*math.cos(3.141593 +robot_ori) - (data[0])*math.sin(3.141593 +data[1]+robot_ori) + robot_pos[0] - 0.01*math.sin(3.141593 +robot_ori)
            location[1] += -0.1*math.sin(3.141593 +robot_ori) + (data[0])*math.cos(3.141593 +data[1]+robot_ori) + robot_pos[2] + 0.01*math.cos(3.141593 +robot_ori)
        else:
            location[1] = -(0.1+0.01)*math.cos(1.570797+robot_ori)-(data[0])*math.cos(1.570797+(data[1]+robot_ori))+robot_pos[2]
            location[0] =  (data[0])*math.sin(1.570797+(data[1]+robot_ori)) +robot_pos[0] + (0.1+0.01)*math.sin(1.570797+robot_ori)
    
    location[0] = round(location[0],2)
    location[1] = round(location[1],2)                 
    return location

def object_data(data): #Getting last data required to get object location
    center_pix = khaliod(data) #Getting new centeroid
    final_distance = get_object_distance(data)*math.cos((abs(center_pix[0][1]-72)/72)*0.785398) #Getting object location
    v_angle = ((center_pix[0][0]-72)/72)*0.785398#Getting object angle from robots center axis 
    return final_distance,v_angle

##    
def get_object(image,angle=0):
    human_list = []
    blue_list = []#sanitizers
    store_list = [] 
    loc_list = [] 
    wall = []
    processed_img = image_process(copy.deepcopy(image),cam_width,20,(cam_height-20))

    
    rf_image = rf.getRangeImage()
    for x in range(0,cam_width,4):

        for y in range(20,(cam_height-20),4):

            di = RangeFinder.rangeImageGetDepth(rf_image,rf_width,x,y)

            for  d in range((x-1),(x+2)): #Getting the two neighbouring pixels
                for  b in range((y-1),(y+2)):
                    if (processed_img[d][b][0]>=200  ) : #Getting red pixels .These thresholds were selected by trial and error
                        human_list.append([d,b])
                    elif processed_img[d][b][2]>= 220 and di < 0.45 :#Getting blue pixels 
                        blue_list.append([d,b])
                    elif  processed_img[d][b][1]>= 230 and store_para_v[1]:#Getting green pixels
                        store_list.append([d,b])   
                    elif wall_check_v[0] and di < 0.60 and d>48 and d<96 and b>20 and b<75 and processed_img[d][b][0]<200 and processed_img[d][b][2]< 160 and processed_img[d][b][1]< 230 :
                        wall.append([d,b])  
           
    imageRef = dis.imageNew(processed_img ,Display.RGB)
    dis.imagePaste(imageRef, 0, 0)#Displaying processed frame
    if wall_check_v[0]:
        imageRef2 = dis2.imageNew(processed_img ,Display.RGB)
        dis2.imagePaste(imageRef2, 0, 0)   
    
    if len(human_list) != 0:
        clustered_human = khali_clustering(human_list,4,"R")#clusting pixels
        if clustered_human != None and len(clustered_human) != 0:
            for i in clustered_human:#Getting one cluster
                pix_data = zooming(i)#Enhancing
                final_pix_data = []
                if pix_data == None:
                    continue
                for loc in pix_data:

                    if (loc not in i) or (processed_img[loc[0]][loc[1]][0]<225) :#Sending pixel not in recognized for processing
                        colour_change(processed_img[loc[0]][loc[1]],1)
                        

                    if ((processed_img[loc[0]][loc[1]][0]>225)  )  :#Getting the red pixels
                        
                        final_pix_data.append(loc)  
                       
                                  
                final_pix_data = khali_clustering(final_pix_data,1,"R") #Clustering new image to remove wall pixels
                

                if final_pix_data == None or len(final_pix_data) == 0:
                    continue
                    
                final_pix_data = final_pix_data[0]
                
                le = final_pix_data[-1][0] - final_pix_data[0][0]
                   
                
                numfpixels = len(final_pix_data)
                data = object_data(final_pix_data)
                red,blue = getting_loc(data,angle)#Getting location
                if not(in_list(red,blue,loc_list,numfpixels,"R")):#Checking if a different object or not

                    loc_list.append([red,blue,"R",numfpixels])               
                if ( data[0] !=None) and data[0]<=0.6:#Displaying caught pixels
                    dis.drawPixel(0,0)

                    """
                    for x in i:
                        dis2.drawPixel(x[0],x[1])
                    """
                    for x in final_pix_data:
                
                        dis.drawPixel(x[0],x[1])         
    if len(blue_list) != 0:#Same as above
    
        clustered_senetizer = khali_clustering(blue_list,4,"B")
        if  clustered_senetizer != None and len(clustered_senetizer) != 0:
            for i in clustered_senetizer:
                """
                pix_data = zooming(i)
                final_pix_data = []
                if pix_data == None:
                    continue
                for loc in pix_data:
                    if (loc not in i) or (processed_img[loc[0]][loc[1]][2]<225) :
                        colour_change(processed_img[loc[0]][loc[1]],1)
                    di = RangeFinder.rangeImageGetDepth(rf_image,rf_width,loc[0],loc[1])
                    if ((processed_img[loc[0]][loc[1]][2]>225) and di<0.45  )  :
                        
                        final_pix_data.append(loc)  
                final_pix_data = khali_clustering(final_pix_data,1)
                if final_pix_data == None or len(final_pix_data) == 0:
                    continue
                    
                final_pix_data = final_pix_data[0]
                """
                data = object_data(i)#data = object_data(final_pix_data)
                #le = final_pix_data[-1][0] - final_pix_data[0][0]
                numfpixels = len(i)#numfpixels = len(final_pix_data)
                red,blue = getting_loc(data,angle)
                if not(in_list(red,blue,loc_list,numfpixels,"B")):
                    loc_list.append([red,blue,"B",numfpixels])               
                if ( data[0] !=None) and data[0]<=0.6:

                    """
                    for x in i:
                        dis2.drawPixel(x[0],x[1])
                    """
                    for x in i:
                
                        dis.drawPixel(x[0],x[1]) 
                        
    if len(store_list) != 0:
    
        clustered_store = khali_clustering(store_list,4,"G")
        if  clustered_store != None and len(clustered_store) != 0:
            for i in clustered_store:
                data = object_data(i)
                
                if data[0] != None and data[0]<=1 :
                    red,blue = getting_loc(data,angle)
                    loc_list.append([red,blue,"S",data[0]])
                    for x in i:
                        dis.drawPixel(x[0],x[1])
    if wall_check_v[0]:
        if len(wall) >= 300 :
            
            wall = khali_clustering(wall,4,"W")
            if wall != None and len(wall) != 0:
                for j in wall:


                    if (j[-1][0] -j[0][0] > 20) :     
                        wall_check_v[1] = True
                        for i in j:
                            dis2.drawPixel(i[0],i[1])
                    else:
                        wall_check_v[1] = None        
            else:
                wall_check_v[1] = None
        else:
            wall_check_v[1] = None
    return  loc_list   

def in_list(r,b,data,num,colour):#Checking if a differnce object or not and removing humans trousers 
    if len(data)== 0 or (data == math.nan):
        return False
    for i in data:
        if abs(i[0] - r)<=0.05  and  abs(i[1] - b)<=0.05:#################################################
            
            if i[2] == "R" and "B"== colour:
                return True 
            else :
                if i[2]==colour and num>i[3]:
                    data.remove(i)
                    return False
                else:
                    return True    
                    
    return False           



#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


#Keyboard robot handling methods-----------------------------------------------
def forward(ws):
    ws[0] = 3.0
    ws[1] = 3.0
        
def pid(change_speed,ws):
    ws[0] = 1+change_speed
    ws[1] = 1-change_speed
    robot.step(100)

def left(ws):
    ws[0] = 1.0
    ws[1] = -1.0
        
def right(ws):
    ws[0] = -1.0
    ws[1] = 1.0
        
def rev(ws):
    ws[0] = -3.0
    ws[1] = -3.0
                     
def stop(ws):
    #print("in stop")
    wheels[0].setVelocity(0.0)
    wheels[1].setVelocity(0.0)
    wheels[2].setVelocity(0.0)
    wheels[3].setVelocity(0.0)
    
    
#-----------------------------------------------------------------------------------
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def countSteps():
    d = [0.25,0.25]
    lastPoint = [0,0]
    preTurn = 0
    while(True):
    
        if(task_completed[0]):
            return
        
        if not enteredMaze[0]  :#Getting the entering point to maze
            
            r_ori =  getComDir()
            t_a = r_ori[1]
            turned_side = 0
            if lastTurn[0] !=r_ori[0]:
                if lastTurn[0] == 0 or lastTurn[0] == 4:
                    if r_ori[0] - lastTurn[0] >=3:
                        turned_side = -1
                    else:
                        turned_side = 1
                else:
                    if r_ori[0] - lastTurn[0] >0:
                        turned_side = 1
                    else:
                        turned_side = -1   
            temp_angle  = 0
            if lastTurn[0] == 0 or lastTurn[0] == 4:
                temp_angle = 1.570797
            elif lastTurn[0] == 2:
                temp_angle = -1.570797
            elif  lastTurn[0] == 1:
                temp_angle = 1.570797*2
                if t_a<0:
                    t_a = -t_a
            if turned_side == 0:               
                l = 0.1*math.cos(abs(t_a-temp_angle)) 
                m=round((ds[0].getValue()*50/100000 + ds[1].getValue()*50/100000)+l,2)
            else:
                m=round((ds[0].getValue()*50/100000 + ds[1].getValue()*50/100000),2)      
            if m <= 0.5:

                r_pos =  gps.getValues()
                ds_l = ds[0].getValue()*50/100000
                ds_r = ds[1].getValue()*50/100000

                entered_point = [r_pos[0],r_pos[2]]
                forward = 0.1
                side_ways = 0.05
       
                    
                if lastTurn[0] !=r_ori[0]:

                    length = ((0.1**2+0.05**2)**0.5)
                    
                        
                    angle = abs(math.tanh(0.5)+abs(r_ori[1]-temp_angle))
                    angle_2 = abs(math.tanh(0.5)-abs(t_a-temp_angle))
                    forward = length*math.cos(angle_2)
                    side_ways = length*math.sin(angle_2) 

                    
                if  lastTurn[0] == 0 or lastTurn[0] == 4:
                    entered_point[1] += forward
                    if  turned_side <= 0 :
                       
                        entered_point[0] += ds_l + side_ways
                    else:

           
                        entered_point[0] += ds_l - side_ways    
 
                            
                elif lastTurn[0] == 2:
                    entered_point[1] -= forward
                    if  turned_side <= 0 :
                        entered_point[0] -= ds_l + side_ways
                    else:
                        entered_point[0] -= (ds_l-side_ways)
                elif lastTurn[0] == 1:
                    entered_point[0] -= forward
                    if  turned_side <= 0 :
                        entered_point[1] += ds_l + side_ways
                    else:
                        entered_point[1] += (ds_l-side_ways)
                else:
                    entered_point[0] += forward
                    if turned_side <= 0 :
                        entered_point[1] -= ds_l + side_ways
                    else:
                        entered_point[1] -= (ds_l-side_ways)    
                                 
                entered_point[0] = round(entered_point[0],2)
                entered_point[1] = round(entered_point[1],2)
                print("enterpoint",entered_point)                                           
                enteredMaze[0] = True
                enteredMaze[1]  = entered_point
                turn_ds(lastTurn[0],True)
                slide_ds("B")
                robot.step(500)

        else:
            midJ = False
            dChange = False
            if(getComDir()[0] >= 0):
                com_dir = round(getComDir()[0])
            else:
                com_dir = 0

            cord = [round(n,3) for n in gps.getValues()]
            del(cord[1])

            if(not path_started[0] and enteredMaze[1] != None ):
                
                sp1 = enteredMaze[1].copy()
                if(lastTurn[0] == 0):
                    sp1[0] -= 0.25
                    sp1[1] -= 0.25
                elif(lastTurn[0] == 1):
                    sp1[1] -= 0.25
                    sp1[0] += 0.25
                elif(lastTurn[0] == 2):
                    sp1[0] += 0.25
                    sp1[1] += 0.25
                elif(lastTurn[0] == 3):
                    sp1[1] += 0.25
                    sp1[0] -= 0.25
                

                path_started[0] = True

                route1[0] = mazemap.Route()

                lastPoint = sp1
                
                
            s = 1
            dn = 1
            
            if(lastTurn[0]%2 ==0):
                yCord = cord[1]
            else:
                yCord = cord[0]
                s = 0
                dn = 0
                
            if(lastTurn[0] != preTurn):
                 d[dn] = 0.25
            
            yCord = round(yCord,2)
            if(path_started[0]):
                c = (round(yCord - lastPoint[s],2)% d[dn] == 0)
                
                if(abs(yCord - lastPoint[s]) == 0): midJ = True

                if(c and (round(s2[1],1) != round(yCord,1)) and not midJ and not mid_block_reached[0]):
                    if(d[dn] != 0.5): d[dn] = 0.5
                    s2[0] += 1
                    route1[0].add_block(lastTurn[0])
                    robot.step(100)
                    if(storeReached[0]):
                        if(storeReached[0] and len(returnRoute[0]) > 0 and returnRoute[0][0][0] == 'T' ):
                            corner_cam_on[0] = True

                        elif(storeReached[0] and len(returnRoute[0]) > 0 and returnRoute[0][0][2] == 'M'):
                            corner_cam_on[0] = False

                        
                        returnRoute[0].pop(0)

                    s2[1] = yCord
                    if(steps_to_go[0] > 0):
                        steps_to_go[0] -= 1

                    if(steps_to_go[0] == 0 and not mid_block_reached[0] and not storeReached[0]):
                        mid_block_reached[0] = True
                        corner_cam_on[0] = False

                        if(returnRoute[0] == []):
                            map1 = mazemap.createMap(route1[0].route)

                            returnRoute[0] = mazemap.placeMen(map1,men_list[0],map_start_point,route1[0].route)

                    lastPoint = [round(cord[0],2),round(cord[1],2)]
                    preTurn = lastTurn[0]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   


def robotMoves():
    processNum = 1
    obstacleFound = False
    dirc = 1
    
    t = 0
    pp_exist = False
    obsPos_changed = False

    spp_exist = False
    sanPos_changed = False
    
    
    
    while robot.step(timestep) != -1:
        

        if(not fpt[0]):
            takePic()
            fpt[0] = True
    
        
        
        if((ds[0].getValue() < 1000 or ds[1].getValue() < 1000) and not enteredMaze[0] and enterPoint == []):
            cordS = [gps.getValues()[n] for n in range(0,3,2)]
            for x in range(2):
                enterPoint.append(round(cordS[x],2))

        if(mid_block_reached[0] and not storeReached[0]):
            store_para_v[0] = True
            stop(ws)
            finding_store(lastTurn,store_para_v)
            mid_block_reached[0] = False
            s2[0] += 1
            returnRoute[0].pop(0)
            steps_to_go[0] -= 1


        dsl = [ds[n].getValue() for n in range(3)]

        if(dsl[0] == 1000 and dsl[1] == 1000 and dsl[2] < 1000):
            inJ[0] = True

        if(task_completed[0]):
            wheels[0].setVelocity(0.0)
            wheels[1].setVelocity(0.0)
            wheels[2].setVelocity(0.0)
            wheels[3].setVelocity(0.0)
            return
            
        if(in_junc_cen[0] and not cont_avoid[0]):
            stop(ws)
            wheels[0].setVelocity(0.0)
            wheels[1].setVelocity(0.0)
            wheels[2].setVelocity(0.0)
            wheels[3].setVelocity(0.0)
            cd = round(getComDir()[0])%4
            robot.step(2000)
            q = -1
            if((cd != 0 and cd > jTurn[0]) or (cd == 0 and jTurn[0] % 4 == 3)):
                q = 1
            sanPos[0] = []
            obsPos[0] = []
            if(pp_exist):
                del(pp)
                pp_exist = False
            if(spp_exist):
                del(spp)
                spp_exist = False
            if(corner_cam_on[0]):
                cornerPic(q)

            juncTurn(jTurn[0])

            sanPos[0] = []
            obsPos[0] = []

            if(pp_exist):
                del(pp)
                pp_exist = False
            if(spp_exist):
                del(spp)
                spp_exist = False
            

            if(not obs_in_corner[0]):
                obsPos[0] = []

        elif(cont_avoid[0]):
            forward(ws)

        
        com_dir = lastTurn[0]
        #GPS values ----------------------------------------------------------------

        op = obsPos[0][:2].copy()
        sp = ssp[0][:2].copy()

        cord = [round(n,3) for n in gps.getValues()]
        
        if(abs(math.sqrt((startingPoint[0]-cord[0])**2 + (startingPoint[2]-cord[2])**2 )) <= 0.1 and storeReached[0]):
            task_completed[0] = True

        if(com_dir%2 != 0):
            cord.reverse()
            op.reverse()
            sp.reverse()
        gSpeed = round(gps.getSpeed(),3)

        
        
        if(man_infront[0]):
            cen_dif = abs(cord[0] - op[0])

        if(sani_infront[0]):
            cen_dif_s = abs(cord[0] - sp[0])

            
        if(sani_infront[0]):
            wall_dis[0] = 80
            

        elif(man_infront[0]):
            wall_dis[0] = 420
            if(cen_dif <= 0.05):
                wall_dis[0] = 260
            if(inJ[0]):
                wall_dis[0] = 380
        else:
            wall_dis[0] = 300
        t += 1
        
        rf_image = rf.getRangeImage()
        

        if(ds[2].getValue() <= 600 and not cor_pic_taken[0] and not inSignal[0]):
            if(corner_cam_on[0]):
                cornerPic()
            if(found_obs_far[0]):
                pass

            forward(ws)
            
   
        if(obstacleFound):
            stop(ws)
            obsTurn()    
            obstacleFound = False
            spp_exist = False
            forward(ws)
            cor_pic_taken[0] = False
            obs_in_corner[0] = False
            man_infront[0] = False
            sani_infront[0] = False
            movedH[0] = False
            
            
        else:  # read sensors
            
            if(ds[2].getValue() <= wall_dis[0] and not inSignal[0]):
                stop(ws)
                obstacleFound = True

                if(len(obsPos[0]) > 0 and confirm_mif[0] == None and man_infront[0]):
                    obstacleFound = False
                if(sani_infront[0] and not done_s1[0]):
                    obstacleFound = False    
        
        if(((not spp_exist) and len(sanPos[0]) > 0) or (sanPos_changed and len(sanPos[0]) > 0)):
            sp2 = sanPos[0][:2].copy()
            if(com_dir%2 != 0):
                sp2.reverse()
            yd = abs(sp2[1] - cord[2])
            xd = abs(sp2[0] - cord[0])
            con3 = (yd <= 0.48 and xd <= 0.24)
            if(con3):
                spp = getSPoints(sanPos[:2])
                if(sanPos_changed):
                    sanPos_changed = False
                spp_exist = True

        if(((not pp_exist) and len(obsPos[0]) > 0 and not (obs_in_corner[0] and not inSignal[0])) or len(obsPos[0]) > 0 and obsPos_changed):
            op2 = obsPos[0][:2].copy()

            if(com_dir%2 != 0):
                op2.reverse()
            yd = abs(op2[1] - cord[2])
            xd = abs(op2[0] - cord[0])
            con2 = (yd <= 0.48 and xd <= 0.24)
            if(con2):
                pp = getPathPoints(obsPos[:2])

                if(obs_in_juction[0]):
                    obs_in_juction[0] = False
                if(obsPos_changed):
                    obsPos_changed = False
                if(obs_in_corner[0]):

                    obs_in_corner[0] = False
                pp_exist = True

    #---------------------------------------------------------------------------

    #turn at obs----------------------------------------------------------------
        if(pp_exist and (ds[2].getValue() > 660)):
            dir = lastTurn[0]%4

            if(mid_block_reached[0]):
                return
            if(dir == 0 or dir == 3):
                if(cord[2] >= pp[0][1]):

                    stop(ws)
                    op1 = obsPos[0][:2]

                    gObsAvoid(pp)
                    forward(ws)
                    op2 = obsPos[0][:2]
                    if(op1 == op2):
                        pass

                    obsPos[0] = []
                    getPathPic() 
                    del(pp)
                    pp_exist = False
            else:
                if(cord[2] <= pp[0][1]):

                    stop(ws)
                    op1 = obsPos[0][:2]

                    gObsAvoid(pp)
                    forward(ws)
                    op2 = obsPos[0][:2]

                    obsPos[0] = [] 
                    del(pp)
                    pp_exist = False   
     
        if((pp_exist or spp_exist) and ds[2].getValue() <= 650 and not sani_incorner[0] and not obs_in_corner[0]):

            if(pp_exist):

                op = obsPos[0][:2]
                if(lastTurn[0]%2 != 0):
                    op.reverse()

                if(lastTurn[0]%4 == 0 or lastTurn[0] %4 == 3 ):
                    if(op[1] > cord[2] and(ds[0].getValue() < 900 or ds[1].getValue() < 900) and enteredMaze[0]):

                        if(confirm_mif[1] == 0):
                            confirm_mif[1] = 1
                            wall_check_v[0] = True
                            takePic()
                            confirm_mif[0] = wall_check_v[1] 

                        if(confirm_mif[0] != None):
                            man_infront[0] = True
                        wall_check_v[0] = False
                        wall_check_v[1] = None
                    else:
                        pass

                else:
                    if(op[1] < cord[2] and(ds[0].getValue() < 900 or ds[1].getValue() < 900) and enteredMaze[0]):
                        if(confirm_mif[0] == None):
                            wall_check_v[0] = True
                            takePic()
                            confirm_mif[0] = wall_check_v[1] 

                        if(confirm_mif[0] != None):
                            man_infront[0] = True
                        wall_check_v[0] = False
                        wall_check_v[1] = None
                    else:
                        pass
                        
                del(pp)
            else:
                
                if(not inSignal[0] and ds[2].getValue()<=600):
                    sani_infront[0] = True
                    takeSanFront()
                    del(spp)
            pp_exist = False
            spp_exist = False
    #-----------------------------------
    #take a sanitizer----------------------------------------------------------
        if(spp_exist):
            op = obsPos[0][:2]
            if(lastTurn[0]%2 != 0):
                op.reverse()
                
            if(lastTurn[0]%4 == 0 or lastTurn[0] %4 == 3 ):
                if(cord[2] >= spp[0]):
                    stop(ws)
                    takeSan(spp)
                    sani_incorner[0] = False
                    forward(ws)
                
                    sanPos[0] = [] 
                    del(spp)
                    spp_exist = False
            else:       
                if(cord[2] <= spp[0]):
                    stop(ws)
                    takeSan(spp)
                    sani_incorner[0] = False
                    forward(ws)
                
                    sanPos[0] = [] 
                    del(spp)
                    spp_exist = False
    #--------------------------------------------------------------------------

        if(not sani_infront[0]):
            l1 = []
            for n in range(3):
                if(ds[n].getValue() <= 900):
                    l1.append(ds[n].getValue())
                    
            if(len(l1) >= 1 and len(l1) < 3 and min(l1) < 240):
                pass

                

        dsl = [round(ds[0].getValue(),2), round(ds[1].getValue(),2)]
        if(True not in status and not mid_block_reached[0]):
            lt = lastTurn[0]%4
            cd = getComDir()[0]
            if(lt == 0 and cd > 3):
                lt = 4
            con1 = abs(lt - cd) > 0.02
            if(con1):
                align()
            if((dsl[0] <= 280 or dsl[1] <= 280) and (round(sum(dsl)) == 760) ):
                gotoCen()
            forward(ws)
        else:
            pass

        wheels[0].setVelocity(ws[0])
        wheels[1].setVelocity(ws[1])
        wheels[2].setVelocity(ws[0])
        wheels[3].setVelocity(ws[1]) 

        ##############################################################################################copy
        if(t%60 == 0 and not obs_in_corner[0]):
            image = cam.getImageArray()
            data = get_object(image)
            #print(data)
            t = 0
            if(len(data) > 0):
                if(data[0][2] == 'R' and len(obsPos[0]) == 0 and len(sanPos[0]) == 0):
                    mazemap.storeMan(men_list[0],0,len(men_list) - 1,data[0][:2])
                    del(data[0][2])
                    obsPos[0] = data[0]
                    obsPos_changed = False
                elif(data[0][2] == 'R' and (len(obsPos[0]) > 0) and not man_infront[0]):
                    if(data[0][3] > obsPos[0][2]):
                        del(data[0][2])
                        obsPos[0] = data[0]
                        obsPos_changed = True
                elif(data[0][2] == 'B' and len(sanPos[0]) == 0):
                    del(data[0][2])
                    sanPos[0] = data[0]
                    sanPos_changed = False
                    ssp[0] = copy.deepcopy(sanPos[0])
                elif(data[0][2] == 'B' and (len(sanPos[0]) > 0) and not sani_incorner[0]):
                    if(data[0][3] > sanPos[0][2]):
                        del(data[0][2])
                        sanPos[0] = data[0]
                        sanPos_changed = True
                        ssp[0] = copy.deepcopy(sanPos[0])

        
        #########################################################################################           
        
        
        pass

print("Place the robot at the entrance")
robot.step(1000)
startingOr = round(getComDir()[0]%4)
lastTurn[0] = startingOr

startingPoint = [round(n,3) for n in gps.getValues()]

t1 = threading.Thread(target=robotMoves)
t2 = threading.Thread(target=comms)
t3 = threading.Thread(target=countSteps)


t2.start()
t1.start()
t3.start()
t1.join()
t2.join()
t3.join()
print("task completed!")
print("Shutting Down Robot")
