import csv

def loadAnnotation(path):
    with open(path) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = next(reader)
        data = list(map(lambda x: int(x), data))
        annotation = [(data[0], data[1]), (data[2], data[3]), data[4]]
        return annotation


def saveAnnotation(path, data):
    def getAnnotationStr(data, index):
        a = data[index]
        return "{},{}".format(a[0], a[1])

    if data is None or data[2] <= 0:
        annot_str = "-1, -1,-1, -1,0"
    else:
        annot_str = "{},{},{}".format(
            getAnnotationStr(data, 0),
            getAnnotationStr(data, 1),
            data[2]
        )

    with open(path, "w") as file:
        file.write(annot_str)

def getAnnotationName(path):
    return path + ".txt"