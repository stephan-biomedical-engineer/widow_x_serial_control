import numpy as np

#input should be a normalized value from 0 to 1
#returns RGB -> colors go from black to white (low to high)
def conv2grayscale(inputValue):
    #convert the raw value to 8 bits
    correctedVal = min(255,inputValue*256)
    return [int(correctedVal)]*3

#input should be a normalized value from 0 to 1
#returns RGB -> colors go from blue to red (low to high)
def conv2colormap(inputValue):

    #convert the raw value to 8 bits
    correctedVal = min(255,inputValue*256)

    #from the corrected value, return RGB
    #RED
    if correctedVal <= 32*4:
        red = 0
    elif correctedVal > 32*4 and correctedVal < 32*6:
        red = ((correctedVal-(32*4))*(255))/((32*6)-(32*4))
    else:
        red = 255

    #BLUE
    if correctedVal >= 32*4:
        blue = 0
    elif correctedVal > (32*2) and correctedVal < 32*4:
        blue = ((correctedVal-(32*2))*(-255))/(32*4-(32*2)) + 255
    else:
        blue = 255

    #GREEN
    if correctedVal >= 0 and correctedVal < (32*2):
        green = ((correctedVal)*(255))/((32*2))
    elif correctedVal >= (32*2) and correctedVal <= (32*6):
        green = 255
    else:
        green = ((correctedVal-(32*6))*(-255))/(255-(32*6)) + 255

    #convert all values to integers
    red = int(red)
    green = int(green)
    blue = int(blue)

    #print(correctedVal,red,green,blue) #debugging
    return [red,green,blue]

#input should be a normalized value from 0 to 1
#returns RGB -> colors go from white to black (yellowish to redish)
def conv2heatmap(inputValue):

    #convert the raw value to 8 bits
    correctedVal = min(255,inputValue*256)

    #from the corrected value, return RGB
    if correctedVal <= 160:
        red = 255
    else:
        red = ((correctedVal-(160))*(-255))/(255-160) + 255

    if correctedVal >= 96:
        blue = 0
    elif correctedVal > 32 and correctedVal < 96:
        blue = ((correctedVal-(32))*(-204))/(96-32)+ 204
    else:
        blue = 204

    if correctedVal <= 160:
        green = 255
    else:
        green = max(0,((correctedVal-(160))*(-255))/(192-160)+ 255)

    red = int(red)
    green = int(green)
    blue = int(blue)

    #print(correctedVal,red,green,blue) #debugging
    return [red,green,blue]
