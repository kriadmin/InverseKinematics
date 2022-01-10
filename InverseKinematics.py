from pygame.math import Vector2
import math
import time
import random

class Robot:
    def __init__(self, lengths, angles, start):
        self.lengths = lengths
        self.length = sum(lengths)
        self.angles = angles
        self.numSegments = len(lengths)
        self.positions = [start]

        for i in range(self.numSegments):
            self.positions.append(Vector2(math.cos(angles[i]),math.sin(angles[i]))*lengths[i] + self.positions[i])
    
    def solvable(self, target):
        return self.length > Vector2(target - self.positions[0]).length()

    def FABRIKSolve(self, target, tolerance):
        if (not self.solvable(target)):
            for i in range(self.numSegments):
                dist = Vector2(self.positions[i] - target).length()
                ratio = self.lengths[i] / dist
                self.positions[i+1] = self.positions[i] * (1 - ratio) + ratio * target
        else:
            iteration = 0
            initial = Vector2(self.positions[0])
            diff = Vector2(self.positions[self.numSegments] - target).length()
            while diff > tolerance:
                iteration += 1
                self.positions[self.numSegments] = Vector2(target)

                #FORWARD REACHING
                for i in reversed(range(self.numSegments)):
                    dist = Vector2(self.positions[i+1] - self.positions[i]).length()
                    ratio = self.lengths[i]/dist

                    self.positions[i] = (1 - ratio) * self.positions[i+1] + ratio * self.positions[i]
                
                #BACKWARD REACHING
                self.positions[0] = Vector2(initial)
                for i in range(self.numSegments):
                    dist = Vector2(self.positions[i] - self.positions[i + 1]).length()
                    ratio = self.lengths[i]/dist

                    self.positions[i+1] = (1 - ratio) * self.positions[i] + ratio * self.positions[i+1]


                diff = Vector2(self.positions[self.numSegments] - target).length()
        return iteration
    def CCDIKSolve(self, target, tolerance):
        it = 0
        if (self.positions[0] - target).length() > self.length: return it
        dist = (self.positions[-1] - target).length()
        while dist > tolerance and it < 2000:
            for i in reversed( range(self.numSegments) ):
                position = self.positions[i]
                dirToEff = Vector2(self.positions[-1] - position)
                dirToTar = Vector2(target - position)
                coss = (dirToTar.dot(dirToEff)/ (dirToTar.length() * dirToEff.length() + 0.00001 ))
                coss = max(-1, min(1, coss))
                angleBetween = math.degrees(math.acos(coss))
                #angleBetween = dirToTar.angle_to(dirToEff)
                for j in range(i,self.numSegments):
                    dist = (self.positions[-1] - target).length()
                    if dist < tolerance: return it
                    pos = self.positions[j+1]
                    vecFromCurr = Vector2(pos - position)
                    news = vecFromCurr.rotate(angleBetween) + position
                    self.positions[j+1] = news
            dist = (self.positions[-1] - target).length()
            it += 1
        return it

lengths = [100] * 100
angles = [90] * 100

angles = [math.radians(x) for x in angles]
width = 1536
start = Vector2(width//2,25)
robotFABR = Robot(lengths, angles, start)
robotCCD = Robot([*lengths],[*angles],start=Vector2(start))

#print(robotCCD.CCDIKSolve(Vector2(width//2 - 300,300),0.001))

randomsx = [random.random() * 300 + 100 for i in range(2000)]
randomsy = [random.random() * 300 + 100 for i in range(2000)]

itFabrik = [0] * 2000
itCCDIK = [0] * 2000

t0 = time.time()

for i in range(2000):
    itFabrik[i] = robotFABR.FABRIKSolve(Vector2(width//2 - randomsx[i], randomsy[i]), 0.01)

t1 = time.time()

fabrtime = t1 - t0
print("done!")
t0 = time.time()

for i in range(2000):
    print(i)
    itCCDIK[i] = robotCCD.CCDIKSolve(Vector2(width//2 - randomsx[i], randomsy[i]), 0.1)

t1 = time.time()

ccdtime = t1 - t0

print(f"FABRIK TIME: {fabrtime}, CCD TIME: {ccdtime}")

print(f"Avg FABRIK iterations = {sum(itFabrik)/2000}")

print(f"Avg CCD iterations = {sum(itCCDIK)/2000}")