a = 2
b = int(input())
c = 9.1
d = "Hello"
i = True

print(a + b)
# 6
print(c - a)
# 7.1
print(a * b)
# 8
print(a / b)
# 0.5
print(a // b)
# 0
print(a % b)
# 2
print("{} world".format(d))
# Hello world
# -*- coding: utf-8 -*-
print("У Васи было " + str(a) + " яблокa")
# У Васи было 2 яблока
if (a == b):
    print("a > b")
elif (a < b):
    print("a < b")
else:
    print("a == b")
# a < b
if i: print("This is True")
# This is True
print(2 == 3)
# False
while a != b:
    a+=1
    print(a)
# 3
# 4
for j in d:
    print(j)
# H
# e
# l
# l
# o
for j in range (1,10,2):
    print(j)
# 0
# 1
# 3
# 5
# 7
# 9
f = []
g = [1, 0, 3]
print(*g)
# 1 0 3
f.append([1, 3])
f.append([1])
print(*f)
# [1, 3] [1]
print(g[0] + g[2])
# 4
def navigate_wait(x = 0, y = 0, z = 1, fraime_id = 'body'):
    print('drone fly to {} {} {} by {}'.format(x, y, z, fraime_id))
def fly_drone(pos):
    for p in pos:
        navigate_wait(x = p[0], y = p[1], z = p[2])
position = [[1 , 1 , 1], [2, 1, 1], [2, 2, 1]]
fly_drone(position)
# drone fly to 1 1 1 by body
# drone fly to 2 1 1 by body
# drone fly to 2 2 1 by body
import time
time.sleep(1)
print(a, b, c, sep='__', end='//\n')
# 2__2__9.1//
f = open("File.txt", "w")
f.write(str(a))
f.write(str(b))
f.close()
f = open("File.txt", "r")
for u in f:
    print(u)
# 44
f.close()