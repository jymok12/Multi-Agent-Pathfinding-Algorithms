import csv
import matplotlib.pyplot as plt

# structure = {
#     'File': 'instances\\test_64.txt', 
#     'cost': '94', 
#     'time': '0.6252617111688927', 
#     'expanded nodes': '656.8227848101266', 
#     'generated nodes': '1105.860759493671', 
#     'maximum nodes': '637.5949367088608', 
#     'Initial collision count': '12.0', 
#     'sampleSize': '79'
# }


def readCsv(fileName):
    result = []
    with open(fileName, newline='') as csvfile:
        spamreader = csv.DictReader(csvfile)
        for row in spamreader:
            row['cost'] = int(row['cost'])
            row['time'] = float(row['time'])
            row['expanded nodes'] = float(row['expanded nodes'])
            row['generated nodes'] = float(row['generated nodes'])
            row['maximum nodes'] = float(row['maximum nodes'])
            row['Initial collision count'] = float(row['Initial collision count'])
            row['sampleSize'] = int(row['sampleSize'])
            result.append(row)
    return result

def getAttributes(resultInstances, attribute):
    result = []
    for instance in resultInstances:
        result.append(instance[attribute])
    return result

def addScatterPlot(instances, attribute_x, attribute_y, labelName, colorName, shape):
    x = getAttributes(instances, attribute_x)
    y = getAttributes(instances, attribute_y)
    plt.scatter(x, y, label= labelName, color= colorName,
                marker= shape, s=30)



cbsAverage = readCsv("CBSresultAverage.csv")
cbsMedian = readCsv("CBSresultMedian.csv")
idcbsAverage = readCsv("IDCBSresultAverage.csv")
idcbsMedian = readCsv("IDCBSresultMedian.csv")

addScatterPlot(cbsMedian, "time", "maximum nodes", "CBS Median", "cyan", "*")
addScatterPlot(cbsAverage, "time", "maximum nodes", "CBS Average", "blue", "*")
addScatterPlot(idcbsMedian, "time", "maximum nodes", "IDCBS Median", "pink", ".")
addScatterPlot(idcbsAverage, "time", "maximum nodes", "IDCBS Average", "red", ".")
plt.xlabel('Time')
plt.ylabel('Memory used')
plt.title('Space vs Time Complexity')
plt.legend()
plt.show()

conflictCutoff = 8

small_cbsAverage = list(filter(lambda x:x['Initial collision count']<conflictCutoff, cbsAverage))
big_cbsAvaerage = list(filter(lambda x:x['Initial collision count']>=conflictCutoff, cbsAverage))
small_cbsMedian = list(filter(lambda x:x['Initial collision count']<conflictCutoff, cbsMedian))
big_cbsMedian = list(filter(lambda x:x['Initial collision count']>=conflictCutoff, cbsMedian))
small_idcbsAverage = list(filter(lambda x:x['Initial collision count']<conflictCutoff, idcbsAverage))
big_idcbsAvaerage = list(filter(lambda x:x['Initial collision count']>=conflictCutoff, idcbsAverage))
small_idcbsMedian = list(filter(lambda x:x['Initial collision count']<conflictCutoff, idcbsMedian))
big_idcbsMedian = list(filter(lambda x:x['Initial collision count']>=conflictCutoff, idcbsMedian))


addScatterPlot(cbsMedian, "Initial collision count", "time", "CBS Median", "cyan", "*")
addScatterPlot(cbsAverage, "Initial collision count", "time", "CBS Average", "blue", "*")
addScatterPlot(idcbsMedian, "Initial collision count", "time", "IDCBS Median", "pink", ".")
addScatterPlot(idcbsAverage, "Initial collision count", "time", "IDCBS Average", "red", ".")

plt.xlabel('Number of Initial Conflicts')
plt.ylabel('Time')
plt.title('Time Complexity over Different Problem Size')
plt.legend()
plt.show()


addScatterPlot(cbsMedian, "Initial collision count", "maximum nodes", "CBS Median", "cyan", "*")
addScatterPlot(cbsAverage, "Initial collision count", "maximum nodes", "CBS Average", "blue", "*")
addScatterPlot(idcbsMedian, "Initial collision count", "maximum nodes", "IDCBS Median", "pink", ".")
addScatterPlot(idcbsAverage, "Initial collision count", "maximum nodes", "IDCBS Average", "red", ".")

plt.xlabel('Number of Initial Conflicts')
plt.ylabel('Memory used')
plt.title('Space Complexity over Different Problem Size')
plt.legend()
plt.show()

