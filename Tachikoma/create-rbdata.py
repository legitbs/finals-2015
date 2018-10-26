import sys, os

outdata = str(sys.argv[1]) + "\n"

for i in xrange(int(sys.argv[1])):
	rbdata = open("shell_%d.prg" % (i + 1),"r").read() + "\n"
	outdata = outdata + str(len(rbdata)) + "\n" + rbdata

open("rbdata","w").write(outdata)
print "Wrote %d bytes to rbdata" % len(outdata)
