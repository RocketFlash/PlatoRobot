from unidecode import unidecode
import codecs

def parse_points(file_name = 'room_318.txt'):
    with codecs.open(file_name,"r", "utf-16") as f:
        content = f.readlines()
        # you may also want to remove whitespace characters like `\n` at the end of each line
        content = [x.strip() for x in content]
        content_x = content[0:len(content):3]
        content_y = content[1:len(content):3]
        content_x = [0.001*float(x.split(';')[3])+5 for x in content_x]
        content_y = [0.001*float(y.split(';')[3])+5 for y in content_y]
        points = list(zip(content_x, content_y))
        return points
