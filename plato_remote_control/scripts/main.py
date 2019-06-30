#!/usr/bin/env python
import json
import signal
import time
import rospy
import rospkg
# import map_to_img_OG as M2I # to get the image made of OccupancyGrid data
import map_to_img_LS as M2I  # the same the below but the img made of LaserScan data
from geometry_msgs.msg import Twist
from plato_control.src.publish_cmd_vel import CommandPublisher
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

PORT = 8081
CAMERA_PORT = 8080

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
cmd = CommandPublisher(pub)

rospack = rospkg.RosPack()
pkgpath = rospack.get_path('plato_remote_control')


def get_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip


def file_to_str(filename):
    fin = open(pkgpath + "/media/" + filename)
    contents = fin.read()
    fin.close()
    return contents


def prepare_page(filename):
    server_ip = get_ip()
    camera_url = server_ip + ":" + str(CAMERA_PORT)
    server_url = server_ip + ":" + str(PORT)
    return file_to_str(filename).format(**locals())


mtypes = {
    ".html": 'text/html',
    ".jpg": 'image/jpg',
    ".png": 'image/png',
    ".js": 'application/javascript',
    ".css": 'text/css',
    ".ico": 'image/x-icon'
}


def getMimeType(p):
    for key in mtypes:
        if p.endswith(key):
            return mtypes[key]
    return 'text/html'


class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        filename = self.path[1:]
        if filename == "":
            filename = "index.html"

        self.send_response(200)
        self.send_header('Content-type', getMimeType(filename))
        self.send_header('Access-Control-Allow-Origin', '*')  # not very safe but works
        self.end_headers()

        if filename.endswith(".html"):
            self.wfile.write(prepare_page(filename))
        else:
            self.wfile.write(file_to_str(filename))

    def do_POST(self):
        if self.path == "/check_connection":
            self.send_response(200)
            self.end_headers()
            self.wfile.write("Connected")
        elif self.path == "/move":
            data_string = self.rfile.read(int(self.headers['Content-Length']))
            data = json.loads(data_string)
            cmd.gas = data["forward"]
            cmd.reverse = data["back"]
            cmd.left = data["left"]
            cmd.right = data["right"]
            # cmd.increase_speed = (buttons[7] == 1)
            # cmd.decrease_speed = (buttons[6] == 1)
            # cmd.increase_angle_speed = (buttons[5] == 1)
            # cmd.decrease_angle_speed = (buttons[4] == 1)
            cmd.publish_command()
            self.send_response(200)
            self.end_headers()


def run(server_class=HTTPServer, handler_class=RequestHandler, port=80):
    server_address = ('', port)
    myServer = server_class(server_address, handler_class)
    prepare_shutdown(myServer)
    m2i = M2I.map2img()

    print("Starting server...")
    myServer.serve_forever()


def prepare_shutdown(server):
    def shutdown_server():
        print('Server shutdowning! Wait please...(max is 60 seconds)')
        start_time = time.time()
        server.shutdown()
        print('Done! In ' + str(time.time() - start_time) + ' seconds')

    def shutdown_handler(signal, frame):
        print('You pressed Ctrl+C!')
        import threading
        t = threading.Thread(target=shutdown_server)
        t.start()

    signal.signal(signal.SIGINT, shutdown_handler)


def main():
    rospy.init_node('plato_remote_control', anonymous=True)

    run(port=PORT)


if __name__ == '__main__':
    main()
