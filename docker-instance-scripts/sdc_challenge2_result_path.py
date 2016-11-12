import vehicle
import matplotlib.pyplot as plt

def loadDatasetInfo(traininSetFolder,steeringAngleFileName):
    imfiles=[]
    steers=[]
    with open(traininSetFolder+ steeringAngleFileName) as f:
        for line in f:
            if line.startswith("frame_id") :
                continue
            fields = line.split(',')
            if len(fields) == 2:
                imfiles.append(line.split(',')[0])
                steers.append(float(line.split(',')[1]))
            else:
                imfiles.append(line.split(' ')[0])
                steers.append(float(line.split(' ')[1]))
    return (imfiles,steers)


def drawDatasetInfo(robot,timestamps,steers,speedVal):
    index = -1
    for steeringAngle in steers:
        index=index+1
        angle.append(steeringAngle) #  * scipy.pi / 180
        speed.append(speedVal)

    robot.sim_Path(speed,angle)             #run in a rectangular path

speed,angle = [],[]

traininSetFolder = "/sharefolder/sdc/ai-world-car-team-c2/mslavescu/submissions/"


fileName = "submission_50-mslavescu.csv"

(imfiles,steers) = loadDatasetInfo(traininSetFolder,fileName)

robot = vehicle.Vehicle(1.5,20)
drawDatasetInfo(robot,imfiles,steers,1)


robot.show(fileName)
plt.show()
print robot.getPose()

