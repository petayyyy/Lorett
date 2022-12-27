#!/usr/bin/env python3 

def geto(id, file):
    f = open(file, 'r')
    for i in f.readlines():
        if(i[0] == '#'):
            continue
        g = list(map(float, i.split("\t")))
        if(id == int(g[0])):
            return g
