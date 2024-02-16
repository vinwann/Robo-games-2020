#import numpy as np
import math

class Route():
    def __init__(self):
        self.route = []

    def add_block(self,direction):
        self.route.append(str(direction))
        
def storeMan(arr,l,r,x):
    #print(arr,r,l)
    if(len(arr) == 0):
        arr.append(x)
        return
    if(r >= l):
        mid = (r + l)//2
        if(arr[mid] == x):
            return mid
        elif(arr[mid][0] > x[0]):
            storeMan(arr,l,mid-1,x)
        else:
            storeMan(arr,mid+1,r,x)
    else:
        arr.insert(l,x)
        dif1 = 0
        dif2 = 0
        if(len(arr) == 1):
            return
        
        if(l > 0):
            dif1 = [abs(arr[l-1][0] - arr[l][0]),abs(arr[l-1][1] - arr[l][1])]
            if(max(dif1) > 0.1):
                pass
            else:
                del(arr[l])
                
        if(l < len(arr) - 1):
            dif2 = [abs(arr[l+1][0] - arr[l][0]),abs(arr[l+1][1] - arr[l][1])]
            if(max(dif2) > 0.1):
                pass
            else:
                del(arr[l])

        return

def createMap(route):
    nX = 0
    nY = 0
      
    max_x = [0,0]
    max_y = [0,0]
    for block in route:
        if(block == '0'):
            nY += 1
            if(max_y[0] < nY):
                max_y[0] = nY
        elif(block == '1'):
            nX += 1
            if(max_x[0] < nX):
                max_x[0] = nX
        elif(block == '2'):
            nY -= 1
            if(nY < 0 and max_y[1] > nY):
                max_y[1] = nY
        elif(block == '3'):
            nX -= 1
            if(nX < 0 and max_x[1] > nX):
                max_x[1] = nX
            


    matW = max_x[0] + abs(max_x[1]) + 1
    matH = max_y[0] + abs(max_y[1]) + 1

    mapMatrix = [['   ']*matW for _ in range(matH)]

    startPoint = (abs(max_x[1]),matH-1 - abs(max_y[1]))
    lp = [startPoint[0],startPoint[1]]

    
    mapMatrix[lp[1]][lp[0]] = ' S '
    
    for block in route:
        if(block == '0'):
            lp[1] -= 1
        elif(block == '2'):
            lp[1] += 1
        elif(block == '1'):
            lp[0] += 1
        elif(block == '3'):
            lp[0] -= 1
            
        mapMatrix[lp[1]][lp[0]] = " " + block + " "

    #print(np.matrix(mapMatrix))
    return mapMatrix

def placeMen(mapMat,menList,sp,rt):
    n = -1
    for b in mapMat:
        n += 1
        if ' S ' in b:
            sb = (n,b.index(' S ')) 

    print("sb",sb)
    menBlockList = []
    for manCord in menList:
        dx = manCord[0] - sp[0]
        dy = manCord[1] - sp[1]

        mb = [0,0]
        if(abs(dx) < 0.5):
            if(dx > 0):
                mb[0] = -1
            else:
                mb[0] = 0
        else:
            if(dx > 0):
                mb[0] = -math.ceil(dx/0.5)
                
            else:
                mb[0] = -math.floor(dx/0.5)

        if(abs(dy) < 0.5):
            if(dy > 0):
                mb[1] = 0
            else:
                mb[1] = -1
        else:
            if(dy > 0):
                mb[1] = math.ceil(dy/0.5)
            else:
                mb[1] = math.floor(dy/0.5)
    
        menBlockList.append(mb)
    
    for m in menBlockList:
        mindex = [sb[0]-m[1],sb[1]+m[0]]
        if(mindex[0] not in range(len(mapMat)) or mindex[1] not in range(len(mapMat[0]))):
            continue
        else:
            if(mapMat[mindex[0]][mindex[1]] != '   '):
                mapMat[mindex[0]][mindex[1]] = mapMat[mindex[0]][mindex[1]][:2] + 'M'
        

    route = []
    inx = [sb[0],sb[1]]

    for b in rt:
        if(b == '0'):
            inx[0] -= 1
        elif(b == '2'):
            inx[0] += 1
        elif(b == '3'):
            inx[1] -= 1
        elif(b == '1'):
            inx[1] += 1
            
        route.append(mapMat[inx[0]][inx[1]])
    
    route.reverse()
    
    for index in range(len(route)):
        if((index+2) in range(len(route))):
            if(route[index][1] != route[index+1][1] and route[index+2][2] == 'M'):
                route[index] = 'T' + route[index][1:]
        
    
    return route
