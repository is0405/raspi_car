import go

start_x = 0.0
final = False
result = go.quinic(start_x)
while( not result[0] ):
    if start_x >= 4.2:
        final = True
        break
    
    start_x += 0.7
    result = go.quinic(start_x)

if not final:
    v_l = result[1]
    v_r = result[2]
    
    for i in range(len(v_l)):
        print("l: " + str(round(v_l[i]*100/0.7)))
        print("r: " + str(round(v_r[i]*100/0.7)))
