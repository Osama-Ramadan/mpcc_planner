def checkOutBoundaries(areaX1, areaY1, areaX2, areaY2):
    return areaX1 < 0 or areaY1 < 0 or areaX2 >= gridSize or areaY2 >= gridSize

def isAreaCompatible(type, oldAreaX1, oldAreaY1, oldAreaX2, oldAreaY2, areaX1, areaY1, areaX2, areaY2):
    global grid
    for y in range(areaY1, areaY2+1):
        for x in range(areaX1, areaX2+1):
            print ("checking point (%s,%s) old area is (%s,%s) (%s,%s) new area is (%s,%s) (%s,%s)" % (x,y,oldAreaX1, oldAreaY1, oldAreaX2, oldAreaY2, areaX1,areaY1,areaX2,areaY2))   
            if x >= oldAreaX1 and x <= oldAreaX2 and y >= oldAreaY1 and y <= oldAreaY2: 
                print ("This area belongs to previous area, won't check")
            else:         
                if grid[y][x].type != type: 
                    print ("false") 
                print ("This area have a different color/type so it's not compatible") 
                return False;
    return True;

def explore(type, x1, y1, x2, y2):
    #Right and bottom
    print ("----- Right and bottom ------")
    areaX1 = x1;
    areaY1 = y1;
    areaX2 = x2+1;
    areaY2 = y2+1;
    if not checkOutBoundaries(areaX1, areaY1, areaX2, areaY2):
        if isAreaCompatible(type, x1, y1, x2, y2, areaX1, areaY1, areaX2, areaY2):      
            #addAnim(areaX1, areaY1, areaX2, areaY2);
            #addMatch(areaX1, areaY1, areaX2, areaY2);
            explore(type, areaX1, areaY1, areaX2, areaY2);

    #Bottom and left    
    print ("----- Bottom and left ------")
    areaX1 = x1-1;
    areaY1 = y1;
    areaX2 = x2;
    areaY2 = y2+1;
    if not checkOutBoundaries(areaX1, areaY1, areaX2, areaY2): 
        if isAreaCompatible(type, x1, y1, x2, y2, areaX1, areaY1, areaX2, areaY2):
            #addAnim(areaX1, areaY1, areaX2, areaY2);
            #addMatch(areaX1, areaY1, areaX2, areaY2);
            explore(type, areaX1, areaY1, areaX2, areaY2);

    #Left and top
    print ("----- Left and top ------")
    areaX1 = x1-1;
    areaY1 = y1-1;
    areaX2 = x2;
    areaY2 = y2;
    if not checkOutBoundaries(areaX1, areaY1, areaX2, areaY2):     
        if isAreaCompatible(type, x1, y1, x2, y2, areaX1, areaY1, areaX2, areaY2):
            #addAnim(areaX1, areaY1, areaX2, areaY2);
            #addMatch(areaX1, areaY1, areaX2, areaY2);
            explore(type, areaX1, areaY1, areaX2, areaY2);

    #Top and right
    print ("----- Top and right ------")
    areaX1 = x1;
    areaY1 = y1-1;
    areaX2 = x2+1;
    areaY2 = y2;
    if not checkOutBoundaries(areaX1, areaY1, areaX2, areaY2):     
        if isAreaCompatible(type, x1, y1, x2, y2, areaX1, areaY1, areaX2, areaY2):
            #addAnim(areaX1, areaY1, areaX2, areaY2);
            #addMatch(areaX1, areaY1, areaX2, areaY2);
            explore(type, areaX1, areaY1, areaX2, areaY2);