f = open("METEOR-M2-2_C4S_2021-08-13T11-29.txt", "r")
k = 0
d = []
for i in f:
    k+=1
    if k >=6:
        #print(i)
        d.append([[float(j) for j in i[:-1].split("\t\t")[1:]][2:]])
        d[k-6].append([int(g) for g in i[:9].split(":")       ])
        
print(d)