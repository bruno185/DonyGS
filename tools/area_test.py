pos_coords=[(381.481464,201.508187,0.0),(381.481464,0.0,0.0),(-18.518536,201.508187,0.0),(-18.518536,0.0,0.0)]
area=0
for i in range(len(pos_coords)):
    x1,y1,_=pos_coords[i]
    x2,y2,_=pos_coords[(i+1)%len(pos_coords)]
    area += x1*y2 - x2*y1
print('XY area',area/2)