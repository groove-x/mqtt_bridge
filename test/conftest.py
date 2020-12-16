import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

import pytest


class ParameterServerMock(BaseHTTPRequestHandler):
    """ mock server for unittest

    Parameter Server API: http://wiki.ros.org/ROS/Parameter%20Server%20API
    Master Slave APIs: http://wiki.ros.org/ROS/Master_Slave_APIs
    """
    def do_POST(self):
        self.send_response(200)
        self.send_header("Content-type", "application/xml")
        self.end_headers()
        body = """<?xml version="1.0"?>
        <methodResponse>
          <params>
            <param><value><int>-1</int></value></param>
            <param><value><string>this is a dummy error response</string></value></param>
            <param><value><int>0</int></value></param>
          </params>
        </methodResponse>"""
        self.wfile.write(body.encode())


@pytest.fixture(scope="session", autouse=True)
def run_mock_parameter_server(request):
    server_address = ('', 11311)
    httpd = HTTPServer(server_address, ParameterServerMock)
    thread = threading.Thread(target=httpd.serve_forever)
    thread.daemon = True
    thread.start()
