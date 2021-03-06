import os, sys
from scipy import stats
from PIL import Image
import matplotlib.pyplot as plot

#
#Author: Brady Testa
#
#Anaylzes the color of the picture w/ respect to the light of the picture. put the pictures in folder 'lightinput'
#and store the light values in light.csv
#
csvFile = open("dataset.csv")
inputFolder = os.listdir("lightinput/")
print ("File list: ")
print(inputFolder)

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
matrix = [[ [0 for a in range(len(inputFolder))] for b in range(rangeY)] for c in range(rangeX)]
results = [0]*(rangeX*rangeY)

linregLight = [0] * len(inputFolder)
linregBV = [0] * len(inputFolder)

for file in inputFolder:
    print("Analysis underway of: " + file)

    toAnalyze = Image.open("lightinput\\" + file)
    picture = toAnalyze.load()
   
    for px in range(ax, bx):
        for py in range(ay, by): #for each pixel

            temp = matrix[px-ax][py-ay][fc]
            
            matrix[px-ax][py-ay][fc] = temp + picture[px, py][2]


    fc = fc+1

for i in range(len(inputFolder)):
    sum = 0.0
    for px in range(ax, bx):
        for py in range(ay, by): #for each pixel
            sum += (matrix[px-ax][py-ay][i])
    sum /= rangeX*rangeY
    linregBV[i] = int(sum)
    linregLight[i] = int(lvm[i][1])

slope, intercept, rv, pv, serr = stats.linregress(linregLight, linregBV)
print("mx+ b: " + str(float(slope)) + "x + " + str(float(intercept)))
print("r-value is: " + str(float(rv)))


def regression(x):
    return float(slope)*float(x) + float(intercept);

line = [0.0]*50
for i in range(len(inputFolder)):
    line[i] = regression(linregLight[i])
    
plot.plot(linregLight,linregBV, 'o', label = "Individual points")
plot.plot(linregLight, line, 'r', label ="line")
plot.legend()
plot.show()





def main():
    response = int(input("Please enter the estimated resistance of the resistor: "))
    low = regression(response*1.1)
    high = regression(response*.9)
    print("ESTIMATION: " + str(low) + ", " + str(high))


main()
