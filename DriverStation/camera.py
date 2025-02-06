import http.server
import socketserver
import threading

PORT = 8080
FOLDER = 'web'

class HttpFileHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=FOLDER, **kwargs)

def run_camera_webserver():
    with socketserver.TCPServer(("", PORT), HttpFileHandler) as httpd:
        print("serving at port", PORT)
        httpd.serve_forever()

def start_camera_webserver():
    webserver_thread = threading.Thread(target=run_camera_webserver)
    webserver_thread.setDaemon(True)
    webserver_thread.start()
