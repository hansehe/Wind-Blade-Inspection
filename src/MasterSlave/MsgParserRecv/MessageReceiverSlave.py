'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

from threading import Thread

class MessageReceiverSlave(Thread):
    '''
     @brief Inherits Thread for allowing to receive and send messages simultaneously.
    '''

    def __init__(self, slave):
        '''CONSTRUCTOR'''
        Thread.__init__(self)
        self.__slave    = slave
        self.target     = self.run
        self.daemon     = True
        self.start()

    def run(self):
        '''
         @brief Receive messages to slave continously.
        '''
        stop = False
        while not(stop):
            try:
                request, content = self.__slave.RecvPackage()
                stop = self.__slave.ReceiveRequest(request, content)
            except:
                self.__slave.ForceTermination()
                self.__slave.FlagError()
                raise
