import pygame, sys
from pygame.locals import *
import csv
from bitstring import BitArray, BitStream

#Number of frames per second
FPS = 100

# Sets size of grid
WINDOWWIDTH = 800
WINDOWHEIGHT = 480
CELLWIDTH = 45
CELLHEIGHT = 20
VGRIDXPOS = 50
VGRIDYPOS = 50
VGRIDCOLS = 14+1
VGRIDROWS = 8
CELLXOFFSET = 5
CELLYOFFSET = 5

# Battery graphic positioning
BATPICXPOS = 55
BATPICYPOS = 300
BATPICWID = 140
BATPICHGT = 75
BATPICXOF = 13
BATPICYOF = 50
BATPCTXPOS = 65
BATPCTYPOS = 360

# Key voltages
MINPACK = (2.2 * 112) * 1.1
MAXPACK = (3.7 * 112) * 0.9
DIFFPACK = (MAXPACK-MINPACK)/5

# Set up the colours
BLACK     = (0  ,0  ,0  )
WHITE     = (255,255,255)
LIGHTGRAY = (200,200,200)
BLUE      = (0  ,0  ,255)
GREEN     = (0  ,255,0  )
RED       = (255,0  ,0  )

def drawGrid():

    ### Draw Minor Lines
    for x in range(0, VGRIDCOLS+1, 1): # draw vertical lines
        xpos = VGRIDXPOS+(x*CELLWIDTH)
        ypostop = VGRIDYPOS
        yposbot = VGRIDYPOS + (VGRIDROWS*CELLHEIGHT)
        pygame.draw.line(DISPLAYSURF, BLACK, (xpos,ypostop),(xpos,yposbot))

    for y in range (0, VGRIDROWS+1, 1): # draw horizontal lines
        ypos = VGRIDYPOS+(y*CELLHEIGHT)
        xposlef = VGRIDXPOS
        xposrig = VGRIDXPOS + (VGRIDCOLS*CELLWIDTH)
        pygame.draw.line(DISPLAYSURF, BLACK, (xposlef,ypos), (xposrig, ypos))

    pygame.draw.rect(DISPLAYSURF, BLUE,(VGRIDXPOS,VGRIDYPOS,(VGRIDCOLS*CELLWIDTH),(VGRIDROWS*CELLHEIGHT)),2)
    pygame.draw.rect(DISPLAYSURF, BLUE, (VGRIDXPOS,VGRIDYPOS,((VGRIDCOLS-1)*CELLWIDTH),(VGRIDROWS*CELLHEIGHT)),2)

    return None

# display a voltage value in a cell
def displayVolt(volts,modulenum,cellnum):
    xpos = VGRIDXPOS + ((cellnum -1)*CELLWIDTH) + CELLXOFFSET
    ypos = VGRIDYPOS + ((modulenum-1)*CELLHEIGHT) + CELLYOFFSET
    populateCells(round(volts,3),xpos,ypos,'small')

# display a temperature value for a module
def displayModuleTemp(temp,modulenum):
    xpos = VGRIDXPOS + ((VGRIDCOLS-1)*CELLWIDTH) + CELLXOFFSET
    ypos = VGRIDYPOS + ((modulenum-1)*CELLHEIGHT) + CELLYOFFSET
    populateCells(str(temp)+" F",xpos,ypos,'small')

# writes cellData at given x, y co-ordinates   
def populateCells(cellData, x, y,size):
    if size == 'small':
        cellSurf = BASICFONT.render('%s' %(cellData), True, BLACK)
    elif size == 'large':
        cellSurf = LARGEFONT.render('%s' %(cellData), True, WHITE)
    elif size == 'larger':
        cellSurf = LARGERFONT.render('%s' %(cellData), True, BLACK)
        
    cellRect = cellSurf.get_rect()
    cellRect.topleft = (x, y)
    DISPLAYSURF.blit(cellSurf, cellRect)

def main():

    # Open the CSV file with CAN data in it
    linenum = 1
    cellnum = 1
    totalv = 0
    with open('A123 full run.csv', 'r') as f:
        reader = csv.reader(f)
        lines = list(reader)

    # Set up the PyGame display
    global FPSCLOCK, DISPLAYSURF
    pygame.init()
    FPSCLOCK = pygame.time.Clock()
    DISPLAYSURF = pygame.display.set_mode((WINDOWWIDTH,WINDOWHEIGHT))

    # Load the background image
    bg = pygame.image.load("Delorean Side.jpg")

    # Load the battery images
    b100 = pygame.image.load("Battery Graphics/Battery Spectrum 100.png")

    mouseClicked = False
  
    mousex = 0
    mousey = 0

    # initialize the voltage store
    # Refered to as cellvolts[module][cell] zero based indexing
    w, h = 14, 8;
    cellvolts = [[0 for x in range(w)] for y in range(h)]
    # define list for module temps
    moduletemps = [0 for x in range(h)]
    # define total pack voltage
    totalpackvolts = 369.25
    # define pack temp probs
    packtempa = 0
    packtempb = 0
    
    pygame.display.set_caption('Battery Data')

    global BASICFONT, BASICFONTSIZE, LARGEFONT, LARGEFONTSIZE, LARGERFONT, LARGERFONTSIZE
    BASICFONTSIZE = 14
    LARGEFONTSIZE = 55
    LARGERFONTSIZE = 55
    BASICFONT = pygame.font.Font('freesansbold.ttf', BASICFONTSIZE)
    LARGEFONT = pygame.font.Font('freesansbold.ttf', LARGEFONTSIZE)
    LARGERFONT = pygame.font.Font('freesansbold.ttf', LARGERFONTSIZE)
    
    # repaints screen
    DISPLAYSURF.fill(WHITE)
    DISPLAYSURF.blit(bg, (0, 0))
    drawGrid()

    # initialize values
    tempbs = ""

    # Useful reference lists
    startcellnum = [0,3,6,9,12]
    group4mods = [[0,0,1],[1,2,2],[3,3,4],[4,5,5],[6,6,7],[7,9,9],[9,9,9],[9,9,9]]

    while True: #main game loop
        mouseClicked = False

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            # mouse movement commands
            elif event.type == MOUSEMOTION:
                mousex, mousey = event.pos

            #Mouse click commands
            elif event.type == MOUSEBUTTONDOWN:
                mousex, mousey = event.pos
                mouseClicked = True

        # If the mouse clicked, load the next Frame of data and update the display
        if mouseClicked == True or mouseClicked == False:

            if linenum >= len(lines):
                pygame.quit()
                sys.exit()
            
            line = lines[linenum]
            linenum += 1

            print(linenum)

            tempbs = str('0x') + line[1]
            b0 = str('0x') + line[6]
            b1 = str('0x') + line[7]
            b2 = str('0x') + line[8]
            b3 = str('0x') + line[9]
            b4 = str('0x') + line[10]
            b5 = str('0x') + line[11]
            b6 = str('0x') + line[12]
            b7 = str('0x') + line[13]

            try:

                #print("Loaded line : %s" % (tempbs))
                framebs = BitArray(tempbs)

                voltageframes = ["0260","0262","0270","0272","0274"]
                dataframes = [5,3,8,8,2]
                if tempbs[-4:] in voltageframes:

                    # Up to 8 bytes of data references (depends on the specific frame id)
                    BAs = [BitArray(b0),BitArray(b1),BitArray(b2),BitArray(b3),BitArray(b4),BitArray(b5),BitArray(b6),BitArray(b7)]
                    ba0 = BitArray(b0)

                    # Extract the counter
                    c1 = ba0[:5]
                    count1 = c1.uint

                    # Get the number of datasets to extract
                    frameindex = voltageframes.index(tempbs[-4:])
                    datacount = dataframes[frameindex]

                    ## print("Line %d : Frame ID %s : Counter : %d" % (linenum, line[1],count1))

                    # Loop for the appropriate number of cycles
                    for dataindex in range(0,datacount):

                        # Get the right BitArray
                        ba = BAs[dataindex]

                        # Extract the status bits
                        bt1 = ba[5]
                        bt2 = ba[6]
                        bt3 = ba[7]

                        # Convert to flags
                        bit1 = "On" if bt1 else "Off"
                        bit2 = "On" if bt2 else "Off"
                        bit3 = "On" if bt3 else "Off"
                        
                        #print("...... Byte %d [%s/%s/%s]" % (dataindex,bit1,bit2,bit3))


                if tempbs[-4:] == "0302":

                    # Up to 8 bytes of data references (depends on the specific frame id)
                    # Byte 1 and 7 contain signalling bits
                    BAs = [BitArray(b0),BitArray(b1),BitArray(b2),BitArray(b3),BitArray(b4),BitArray(b5),BitArray(b6),BitArray(b7)]
                    ba0 = BitArray(b0)
                    ba7 = BitArray(b7)

                    # Get the signal bits
                    bt0 = ba0[7]
                    bt61 = ba7[2]

                    # Convert to flags
                    bit1 = "On" if bt0 else "Off"
                    bit61 = "On" if bt61 else "Off"

                    # Cycle through the temperatures
                    if bit1 == "On":
                        #print("Line %d : Frame ID %s : Temps..." % (linenum, line[1]))

                        # Depending on Bit61 (which bank of data) figure out how many temperatures to report
                        for dataindex in range(1,(3 if bit61 == "On" else 7)):

                            # fetch the byte of temp
                            bat = BAs[dataindex]

                            # Convert to temperature
                            temp = bat.uint - 40

                            #print("...... Block %d : %d F" % (dataindex,temp))

                            # Set values in the temp array
                            if bit61 == "Off":
                                moduletemps[dataindex-1] = temp
                            else:
                                moduletemps[dataindex+5] = temp
     
                if tempbs[-4:] == "0216":

                    # Get the 4 bytes hosting data
                    ba1 = BitArray(b1)
                    ba2 = BitArray(b2)
                    ba3 = BitArray(b3)
                    ba4 = BitArray(b4)

                    # Build the suspected data components
                    n1 = ba3+ba4
                    v1 = ba1[-5:]+ba2
                    volt1 = float(v1.uint) / (8 if v1.uint !=0 else 1)
                    num1 = n1.int

                    #print("Line %d : Frame ID %s : Voltage : %.4f , Integer %d" % (linenum, line[1],volt1,num1))

                    # Set the pack voltage
                    totalpackvolts = volt1

                if tempbs[-4:] == "0460":

                    # 4 bytes host two cell voltage
                    ba0 = BitArray(b0)
                    ba1 = BitArray(b1)
                    ba2 = BitArray(b2)
                    ba3 = BitArray(b3)

                    # Convert to temperature
                    temp1 = float(ba1.uint/2) - 30
                    temp2 = float(ba3.uint/2) - 30

                    #if ba0.uint > 0 and ba2.uint > 0:
                        #print("Line %d : Frame ID %s : Temp#1 : %.1f F , Temp2 : %.1f F" % (linenum, line[1],temp1,temp2))

                    #else:
                        #print("Line %d : Frame ID %s : No data reported" % (linenum, line[1]))

                    # Set the temperature values
                    packtempa = temp1
                    packtempb = temp2

                voltageframes = ["0200","0202","0204","0206","0208"]
                if tempbs[-4:] in voltageframes:

                    ba0 = BitArray(b0)
                    ba1 = BitArray(b1)
                    ba2 = BitArray(b2)
                    ba3 = BitArray(b3)
                    ba4 = BitArray(b4)
                    ba5 = BitArray(b5)
                    ba6 = BitArray(b6)
                    ba7 = BitArray(b7)

                    # which 0-4 group is reporting
                    groupnum = voltageframes.index(tempbs[-4:])
                    # which 0-7 module are we on
                    cellmodule = ba6[:3]
                    modulenum = cellmodule.uint

                    # 5 bits from one byte and 8 bits from the other = 13 bits (see MIT data)
                    v1 = ba0[-5:]+ba1
                    v2 = ba2[-5:]+ba3
                    v3 = ba4+ba5[:5]

                    # Process the 3 cell frames
                    if groupnum != 4:

                        # get the startcell num
                        cell1 = startcellnum[groupnum]
                        cell2 = cell1 + 1
                        cell3 = cell2 + 1

                        #print("%s , %d ,%d : %d,%d,%d" % (tempbs[-4:],modulenum,groupnum,cell1,cell2,cell3))

                        # div by 16 for 100ths of a volt
                        volt1 = float(v1.uint) / (1600 if v1.uint !=0 else 1)
                        volt2 = float(v2.uint) / (1600 if v2.uint !=0 else 1)
                        volt3 = float(v3.uint) / (1600 if v3.uint !=0 else 1)

                        # set the value in voltages array
                        cellvolts[modulenum][cell1] = volt1
                        cellvolts[modulenum][cell2] = volt2
                        cellvolts[modulenum][cell3] = volt3

                    else :      #208  (2 voltages/module)

                        cell1 = group4mods[modulenum][0]
                        cell2 = group4mods[modulenum][1]
                        cell3 = group4mods[modulenum][2]

                        cellstart = startcellnum[groupnum]

                        if cell1 !=9:
                            if cell1 != cell2:
                                cellstart +=1
                            #print("%s , %d ,%d : %d,%d,%d : %d" % (tempbs[-4:],modulenum,groupnum,cell1,cell2,cell3,cellstart))
                            volt1 = float(v1.uint) / (1600 if v1.uint !=0 else 1)                       
                            cellvolts[cell1][cellstart] = volt1
                            cellstart +=1

                        if cell2 !=9:
                            if cell2 != cell1:
                                cellstart = startcellnum[groupnum]
                            #print("%s , %d ,%d : %d,%d,%d : %d" % (tempbs[-4:],modulenum,groupnum,cell1,cell2,cell3,cellstart))
                            volt2 = float(v2.uint) / (1600 if v2.uint !=0 else 1)                       
                            cellvolts[cell2][cellstart] = volt2
                            cellstart +=1

                        if cell3 !=9:
                            if cell3 != cell2:
                                cellstart = startcellnum[groupnum]
                            #print("%s , %d ,%d : %d,%d,%d : %d" % (tempbs[-4:],modulenum,groupnum,cell1,cell2,cell3,cellstart))
                            volt3 = float(v3.uint) / (1600 if v3.uint !=0 else 1)                       
                            cellvolts[cell3][cellstart] = volt3

            except EOFError:
                pass
        
        # repaints screen
        DISPLAYSURF.fill(WHITE)
        DISPLAYSURF.blit(bg, (0, 0))

        # Draw the battery cell status grid
        drawGrid()

        # show the battery status
        DISPLAYSURF.blit(b100,(BATPICXPOS,BATPICYPOS))
        batteryint = int(round((totalpackvolts - MINPACK) / (MAXPACK - MINPACK)*100))
        if batteryint > 100:
            batteryint = 100
        if batteryint < 0:
            batteryint = 0
        batterypct = str(batteryint) + "%"
        batleft = int(batteryint/100*BATPICWID)
        batwidth = int((100-batteryint)/100*BATPICWID)
        pygame.draw.rect(DISPLAYSURF, BLACK, [BATPICXPOS+BATPICXOF+batleft, BATPICYPOS+BATPICYOF, batwidth, BATPICHGT])
        populateCells(batterypct,BATPCTXPOS+2,BATPCTYPOS+2,'larger')
        populateCells(batterypct,BATPCTXPOS,BATPCTYPOS,'large')

        # display the voltages
        for cell in range(14):
            for module in range(8):
                displayVolt(cellvolts[module][cell],module+1,cell+1)

        # display module temperatures
        for module in range(8):
            displayModuleTemp(moduletemps[module],module+1)

        # Display the summary pack data
        xpos = VGRIDXPOS + CELLXOFFSET
        ypos = VGRIDYPOS + ((9*CELLHEIGHT) + CELLYOFFSET)
        populateCells(('Total Pack   ' + str(totalpackvolts) + ' v'),xpos,ypos,'small')
        ypos = ypos + CELLHEIGHT + CELLYOFFSET
        populateCells(('Temp A   ' + str(packtempa) + ' F'),xpos,ypos,'small')
        ypos = ypos + CELLHEIGHT + CELLYOFFSET
        populateCells(('Temp B   ' + str(packtempb) + ' F'),xpos,ypos,'small')

        pygame.display.update()    
        FPSCLOCK.tick(FPS)

if __name__=='__main__':
    main()
