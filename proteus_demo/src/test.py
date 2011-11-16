import rawlasertwist

scan = range(30)
velo = rawlasertwist.rawlasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == 1.0)
scan.reverse()
velo = rawlasertwist.rawlasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == -1.0)
scan = [3.0 for i in range(30)]
velo = rawlasertwist.rawlasertwist(scan)
assert(velo[0] == 1.0 and velo[1] == 0.0)
scan = range(10)
velo = rawlasertwist.rawlasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == 0.0)

