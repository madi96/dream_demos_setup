#!/usr/bin/env python

import threading
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.httpserver
import json
import roslib
import rospy

from std_msgs.msg import String

class Interface():

    callbacks=[]

    def register(self, callback):
        print 'registering a callback'
        self.callbacks.append(callback)
        
    def unregister(self, callback):
        self.callbacks.remove(callback)

    def notifyInterface(self, msgToInterface):
        for callback in self.callbacks:
            print 'msgToInterface',msgToInterface
            callback(msgToInterface)
    


class MainHandler(tornado.web.RequestHandler):

    def get(self):
        loader = tornado.template.Loader(".")
        self.write(loader.load("./src/dream_demos_setup/web/index.html").generate())


class WebSocketHandler(tornado.websocket.WebSocketHandler):

    def open(self):
        print "New client !"
        self.application.interface.register(self.upadateInterfaceCallback)
        pass
  
    def on_message(self, message):
        self.write_message(u"Your message was: " + message)
        print message
  
    def on_close(self):
        print "Goodbye, see you later !"
        self.application.interface.unregister(self.upadateInterfaceCallback)
        pass
    
    def check_origin(self, origin):
        return True

    def upadateInterfaceCallback(self, data):
        self.write_message(data)


class Application(tornado.web.Application):

    def __init__(self):
        self.interface = Interface()

        handlers = [(r"/", MainHandler),
                    (r"/ws", WebSocketHandler )
        ]

        tornado.web.Application.__init__(self, handlers, {})


class ServerNode():
    def __init__(self, tornadoApp):
        self.application=tornadoApp

    def updateInterfaceCallback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "Updating the interface with %s", data.data)
        # infoList= data.data.split(":")
        # if len(infoList) == 2:
        #     dataDict = {"type":infoList[0], "firstValue":infoList[1]}
        # elif len(infoList) == 3:
        #     dataDict = {"type":infoList[0], "firstValue":infoList[1], "secondValue": infoList[2]}
        # # else error msg not enough args
        # txt = json.dumps(dataDict)
        self.application.interface.notifyInterface(data.data)

    def server(self):
        rospy.init_node('server', anonymous=True)
        rospy.Subscriber("interface", String, self.updateInterfaceCallback)
        rospy.spin()


def launchWebserver():    
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    app = Application()
    app.listen(8888)

    serverThread=threading.Thread(target=launchWebserver)
    serverThread.daemon = True
    serverThread.start()
    #tornado.ioloop.IOLoop.current().start()
    server_node = ServerNode(app)
    print "Server Launched on localhost:8888"
    server_node.server()
    #threading.Thread(target=server_node.server).start()
