E_RAD = 6371000
import math

def gps_to_cartesian(filename, outputfile):
    with open(filename) as file:
        lines = file.readlines()

    point0 = lines[1].split()

    uav_height = float(lines[0])

    points = [[0,0, math.sqrt(math.pow(float(point0[2]),2) - math.pow(uav_height, 2))]]

    for i in range(2, len(lines)):
        point = lines[i].split()
        cpoint = [(E_RAD*math.radians(float(point[1]) - float(point0[1]))*math.cos(math.radians(float(point[0]) + float(point0[0]))/2)) , E_RAD*math.radians(float(point[0]) - float(point0[0])), "" if len(point) == 2 else math.sqrt(math.pow(float(point[2]),2) - math.pow(uav_height, 2)) ]

        points.append(cpoint)

    with open(outputfile, "w") as file:
        file.write(lines[0])
        for i in range(0, len(points)):
            file.write("{x} {y} {r}\n".format(x = points[i][0], y = points[i][1], r = points[i][2]))

if __name__ == "__main__":
    gps_to_cartesian("TestLines_RawGPS.txt", "TestLines_Cartesian.txt")
