import scipy.misc

with open("interpolated.csv") as f:
    for line in f:
        fields = line.split(',')
        if(len(fields)>6 and fields[4]=="center_camera"):
            #if abs(float(line.split()[1])) > 0.1745: #10 deg #0.12: # 5.50 degrees
            #   continue
            #xs.append("driving_dataset/" + line.split()[0])o
            fileName = fields[1]+".jpg"
            #imgResized = scipy.misc.imresize(scipy.misc.imread("center/"+fileName)[-150:], [66, 200]) / 255.0
            imgResized = scipy.misc.imresize(scipy.misc.imread("center/"+fileName), [120, 160]) / 255.0
            scipy.misc.imsave("center-resized/"+fileName,imgResized)

