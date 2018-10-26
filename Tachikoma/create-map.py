import sys, os
from PIL import Image

FieldSize = 16390
RBSize = 33
Scale = 0.25
im = Image.new("RGB", (int(FieldSize * Scale) + 1, int(FieldSize * Scale) + 1))

data = open("log","r").read().split("\n")

RBColor = [0x888888, 0xff0000, 0x0000ff, 0xff00ff, 0xffff00, 0x00ffff]

Start = 1

RBLocs = [0]*3

for i in data:
	if i[0:6] == "Robot " and i.find("start") != -1:
		RBData = i.split(" ")
		X = int(float(RBData[4]) * Scale)
		Y = int(((FieldSize - float(RBData[5])) * Scale))
		RBID = int(RBData[1])
		X = X - int(RBSize * Scale / 2)
		Y = Y - int(RBSize * Scale / 2)
		for CurX in xrange(X, X+int(RBSize * Scale)):
			for CurY in xrange(Y, Y+int(RBSize * Scale)):
				im.putpixel((CurX, CurY), RBColor[RBID])

	if i[0:4] == 'RD 2':
		Start = 1

	if not Start:
		continue

	if i[0:2] == "M ":
		RBData = i.split(" ")
		X = int(float(RBData[2]) * Scale)
		Y = int((FieldSize - float(RBData[3])) * Scale)
		im.putpixel((X, Y), 0xffffff)

	elif i[0:2] == "R ":
		RBData = i.split(" ")
		X = int(float(RBData[3]) * Scale)
		Y = int(((FieldSize - float(RBData[4])) * Scale))
		RBID = int(RBData[1])
		X = X - int(RBSize * Scale / 2)
		Y = Y - int(RBSize * Scale / 2)
		if X < 0:
			X = 0
		if Y < 0:
			Y = 0
		for CurX in xrange(X, X+int(RBSize * Scale)):
			for CurY in xrange(Y, Y+int(RBSize * Scale)):
				im.putpixel((CurX, CurY), RBColor[RBID])

im.save("map.png")

