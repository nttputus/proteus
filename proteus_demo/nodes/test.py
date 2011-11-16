import rawlasertwist

# test C version (via Swig Python/C-API)

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

print("TEST rawlasertwist.rawlasertwist OK")

# test python version

def lasertwist(ranges):
    velocity = [0.0, 0.0] # v,omega

    if len(ranges) >= 30:
        halt = False
        mid = len(ranges) // 2
        # halt if an object is less than 2m in a 30deg angle
        for distance in ranges[mid-15:mid+15]:
            if distance < 2:
                halt = True
                break
        if halt:
            # we go to the highest-range side scanned
            if sum(ranges[:mid]) > sum(ranges[mid:]):
                velocity[1] = -1
            else:
                velocity[1] = 1
        else:
            velocity[0] = 1

    return velocity

scan = range(30)
velo = lasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == 1.0)
scan.reverse()
velo = lasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == -1.0)
scan = [3.0 for i in range(30)]
velo = lasertwist(scan)
assert(velo[0] == 1.0 and velo[1] == 0.0)
scan = range(10)
velo = lasertwist(scan)
assert(velo[0] == 0.0 and velo[1] == 0.0)

print("TEST python lasertwist OK")

# bench stress test

import time

scan = range(270*4)

t0 = time.time()
for i in range(10000):
    velo = rawlasertwist.rawlasertwist(scan)
t1 = time.time()
for i in range(10000):
    velo = lasertwist(scan)
t2 = time.time()

print("scan = range(270*4)                   %f %f %f"%(t0, t1, t2))
print("rawlasertwist.rawlasertwist : %f sec"%(t1-t0))
print("lasertwist                  : %f sec"%(t2-t1))

scan = [1.0 for i in range(270*4)]

t0 = time.time()
for i in range(10000):
    velo = rawlasertwist.rawlasertwist(scan)
t1 = time.time()
for i in range(10000):
    velo = lasertwist(scan)
t2 = time.time()

print("scan = [1.0 for i in range(270*4)]    %f %f %f"%(t0, t1, t2))
print("rawlasertwist.rawlasertwist : %f sec"%(t1-t0))
print("lasertwist                  : %f sec"%(t2-t1))

