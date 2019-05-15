import os, sys
from scipy import stats
from PIL import Image
import matplotlib.pyplot as plot
import math

#
#Author: Brady Testa
#
#Anaylzes the color of the picture w/ respect to the light of the picture. put the pictures in folder 'lightinput'
#and store the light values in light.csv
#
csvFile = open("dataset2.csv")
inputFolder = os.listdir("lightinput2/")
print ("File list: ")
print(inputFolder)

CH = 3 #channels
ax = 650
ay = 370
bx = 690
by = 400
rangeX= bx-ax
rangeY= by-ay

lvm = [[]] * len(inputFolder)

for i in range(len(inputFolder)):
    lvm[i] = csvFile.readline().replace("\n", "").split(',')

lvm = sorted(lvm, key=lambda tup: tup[0])
fc =0

lightValues = [0] * len(inputFolder)
matrix = [[ [[0 for d in range(CH)] for a in range(len(inputFolder))] for b in range(rangeY)] for c in range(rangeX)] 
results = [0]*(rangeX*rangeY)*CH


linregLight = [[[0] for a in range(len(inputFolder))] for b in range(CH)] 
linregBV = [[[0]for a in range(len(inputFolder)) ]for b in range(CH) ] 

for file in inputFolder:
    print("Analysis underway of: " + file)

    toAnalyze = Image.open("lightinput2\\" + file)
    picture = toAnalyze.load()
   
    for px in range(ax, bx):
        for py in range(ay, by): #for each pixel
            for pc in range(0,CH):

                matrix[px-ax][py-ay][fc][pc] = matrix[px-ax][py-ay][fc][pc] + picture[px, py][pc]

    fc = fc+1

for i in range(len(inputFolder)):
    sum = [0] * CH
    for pc in range(0,CH):
        for px in range(ax, bx):
            for py in range(ay, by): #for each pixel
                sum[pc] += (matrix[px-ax][py-ay][i][pc])
        sum[pc] /= rangeX*rangeY
        linregBV[pc][i] = (int(sum[pc]))
        linregLight[pc][i] = (int(lvm[i][1]))
        print(str(linregLight[pc][i]) + " " + str(linregBV[pc][i]))
slope= [0] * CH;
intercept= [0] * CH
rv= [0] * CH
pv= [0] * CH
serr = [0] * CH
for a in range(0,CH):
    slope[a], intercept[a], rv[a], pv[a], serr[a] = stats.linregress(linregLight[a], linregBV[a])
    print("CHANNEL: " + str(a))
    print("mx+ b: " + str(float(slope[a])) + "x + " + str(float(intercept[a])))
    print("r-value is: " + str(float(rv[a])))


def regression(x,a):
    return float(slope[a])*float(x) + float(intercept[a]);

line = [[0.0 for a in range(len(inputFolder))] for b in range(CH)]
for j in range(CH):
    for i in range(len(inputFolder)):
        line[j][i] = regression(linregLight[j][i], j)
        

plot.plot(linregLight[0],linregBV[0], 'o', label = "Red")
plot.plot(linregLight[1],linregBV[1], 'o', label = "Green")
plot.plot(linregLight[2],linregBV[2], 'o', label = "Blue")
plot.plot(linregLight[0], line[0], 'r', label ="Red")
plot.plot(linregLight[1], line[1], 'r', label ="Green")
plot.plot(linregLight[2], line[2], 'r', label ="Blue")
plot.legend()
plot.show()




