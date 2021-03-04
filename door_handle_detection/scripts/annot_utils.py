import csv

def loadAnnotation(path):
    with open(path) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = next(reader)
        data = list(map(lambda x: int(x), data))
        annotation = [(data[0], data[1]), (data[2], data[3]), data[4]]
        return annotation

def getAnnotationName(path):
    return path + ".txt"