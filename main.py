from time import sleep, time
import cv2 as cv
import numpy as np
import pyautogui

def findClickRectangles(haystackImg, needleImg, threshold = 0.6):
    # Get dimensions of needle image
    needleW = needleImg.shape[1]
    needleH = needleImg.shape[0]

    # Get results
    result = cv.matchTemplate(haystackImg, needleImg, cv.TM_CCOEFF_NORMED);

    # Filter locations
    locations = np.where(result >= threshold)
    # zip locations https://www.youtube.com/watch?v=ffRYijPR8pk #NEED TO UNDERSTND THIS LATER
    locations = list(zip(*locations[::-1]))

    # Draw rectangles [x, y, w, h]
    rectangles = []
    for loc in locations:
        rect = [int(loc[0]), int(loc[1]), needleW, needleH]
        # appending twice because groupRectangles() needs two or more rectangles to create a group
        # if there is only one rectangle, it won't create a group
        rectangles.append(rect)
        rectangles.append(rect)

    rectangles, _ = cv.groupRectangles(rectangles, 1, 0.5)
    return rectangles

def findClickPointers(rectangles, haystackImg, DEBUG = False):
    clickPoints = []
    if len(rectangles) == 0:
        return clickPoints
    
    lineColor = (0, 255, 0)
    lineType = cv.LINE_4

    # Loop over all locations and draw box
    for (x, y, w, h) in rectangles:
        # Determine box position
        topLeft = (x, y)
        bottomRight = (x + w, y + h)
        # Draw box
        if DEBUG:
            cv.rectangle(haystackImg, topLeft, bottomRight, lineColor, lineType)

        # Determine center of square
        centerX = x + int(w / 2)
        centerY = y + int(h / 2)
        # Draw center point
        center = (centerX, centerY)
        clickPoints.append(center)
        if DEBUG:
            cv.circle(haystackImg, center, 5, lineColor, lineType)
    
    if DEBUG:
        cv.imshow('Matches', haystackImg)
        cv.waitKey()
        cv.destroyAllWindows()

    return clickPoints

def getScreenshot():
    # If want to improve later https://www.youtube.com/watch?v=WymCpVUPWQ4&t=2s
    screenshoot = np.array(pyautogui.screenshot())
    screenshoot = cv.cvtColor(screenshoot, cv.COLOR_RGB2BGR)
    return screenshoot

def fileToCommandList(path):
    commands = []
    with open(path) as f:
        for line in f:
            if line[0] == '#':
                continue
            command = line.strip().split(',')
            commands.append(command)
    return commands

def buildQueue():
    rawCommands = fileToCommandList('queue.txt')
    commands = []

    for command in rawCommands:
        if command[0] != "macro":
            commands.append(command)
        else:
            macroPath, macroRepeat = command[1], int(command[2])
            macroCommands = fileToCommandList(macroPath)
            for i in range(macroRepeat):
                for macroCommand in macroCommands:
                    commands.append(macroCommand)

    return commands

queueActions = buildQueue()

while True:
    if len(queueActions) == 0:
        print("No action in the list, skipping...")
        sleep(1)
        continue

    print("Action: " + str(queueActions[0]), "Left: " + str(len(queueActions)))
    if queueActions[0][0] == "click":
        screenshoot = getScreenshot()

        needle = cv.imread(queueActions[0][1], cv.IMREAD_UNCHANGED)
        rectangles = findClickRectangles(screenshoot, needle, 0.8)
        points = findClickPointers(rectangles, screenshoot)
        if len(points) == 0:
            # If not found check for fallback image
            sleep(3)
            if not queueActions[0][2]:
                print("Not found, aborting...")
                break
            needle = cv.imread(queueActions[0][2], cv.IMREAD_UNCHANGED)
            rectangles = findClickRectangles(screenshoot, needle, 0.8)
            points = findClickPointers(rectangles, screenshoot)
            if len(points) == 0:
                print("Not found, aborting...")
                break
        pyautogui.click(points[0])
        queueActions.pop(0)

    elif queueActions[0][0] == "sleep":
        sleep(int(queueActions[0][1]))
        queueActions.pop(0)