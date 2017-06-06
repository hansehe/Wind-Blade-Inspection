'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import socket, sys, os, timeit
from src.bin.tools import RunThread
from Requests import Requests
from MsgParserRecv.MessageReceiverSlave import MessageReceiverSlave
from MsgParserRecv.MessageParser import MessageParser

'''
 @brief Slave class 

 @param settings_inst (TCP settings)
'''
class Slave(MessageParser, Requests):
    def __init__(self, settings_inst, subclass):
        '''CONSTRUCTOR'''
        MessageParser.__init__(self)
        Requests.__init__(self, False)
        self.__host                 = settings_inst.GetSettings('master_ip')
        self.__server_port          = settings_inst.GetSettings('port')
        self.__buffer_size          = settings_inst.GetSettings('slave_buffer_size')
        self.__max_send_size        = settings_inst.GetSettings('master_buffer_size')
        self.__timeout              = settings_inst.GetSettings('tcp_timeout')
        self.__print_payload_info   = settings_inst.GetSettings('print_payload_info')
        self.__terminate            = False
        self.__connected            = False
        self.__error_flag           = False
        self.__subclass             = subclass

    def Connect(self):
        '''
         @brief Connect to master and wait for request
        '''
        # Set up the socket connection to the master
        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Initiate the connection to the master
        start_time = timeit.default_timer()
        while not(self.__connected):
            try:
                self.__connection.connect((self.__host, self.__server_port))
                self.__connected = True
            except socket.error, error:
                elapsed = timeit.default_timer() - start_time
                if elapsed > self.__timeout:
                    self.__terminate = True
                    raise
        MessageReceiverSlave(self)
        
    def Disconnect(self):
        '''
         @brief Disconnect port
        '''
        if self.__connected:
            self.__connection.close()
            self.__connected = False

    def CheckConnected(self):
        '''
         @brief Check if slave is still connected

         @return True/False
        '''
        return self.__connected

    def FlagError(self):
        '''
         @brief Flag error on receiving request from slave
        '''
        self.__error_flag = True

    def CheckErrorFlag(self):
        '''
         @brief Check error flag
        '''
        return self.__error_flag

    def RecvLarge(self, length):
        '''
         @brief Receive large message given by length

         @param length Number of bytes to receive.
        '''
        payload_raw = ''
        while len(payload_raw) < length:
            payload_raw += self.Recv()
        return payload_raw

    def Recv(self):
        '''
         @brief Receive message

         @return payload_raw Received message
        '''
        reads = 0
        payload_raw = ''
        while len(payload_raw) == 0 and reads < 5:
            try:
                payload_raw = self.__connection.recv(self.__buffer_size)
            except:
                self.Disconnect()
                self.__terminate = True
                raise
            reads += 1
        return payload_raw

    def SendLarge(self, payload_raw, max_len=512):
        '''
         @brief Send large message 

         @param payload_raw Message to send
         @param max_len (default=512)
        '''
        if len(payload_raw) > max_len:
            offset = 0
            while offset < len(payload_raw):
                if offset + max_len < len(payload_raw):
                    self.Send(payload_raw[offset:offset+max_len])
                else:
                    self.Send(payload_raw[offset:])
                offset += max_len
        else:
            self.Send(payload_raw)

    def Send(self, payload_raw):
        '''
         @brief Send message 

         @param payload_raw Message to send
        '''
        length = len(payload_raw)
        offset  = 0
        while offset < length:
            try:
                offset = self.__connection.send(payload_raw[offset:])
            except:
                self.Disconnect()
                self.__terminate = True
                raise

    def RecvPackage(self):
        '''
         @brief Receive a package from master
          Raises Exception if:
            No response
            Error response
            response doesn't match request

         @param request Sent request. 
        '''
        payload_raw = self.Recv()
        response, content = self.Parse(payload_raw)
        if response == 'error':
            raise Exception(content)
        elif response == 'response_size':
            self.SendAck()
            payload_raw = self.RecvLarge(content['length'])
            response, content = self.Parse(payload_raw)
        return response, content

    def SendAck(self):
        '''
         @brief Send acknowledge
        '''
        request = 'ack'
        payload = {'request': request, 'content': ''}
        payload_raw = self.DumpJson(payload)
        self.SendLarge(payload_raw, self.__max_send_size)

    def AssertAck(self):
        '''
         @brief Assert acknowledge received
        '''
        request, content = self.RecvPackage()
        if request != 'ack':
            raise Exception('Failed receiving ack - received "{0}" instead'.format(request))

    def SendPayload(self, request, content):
        '''
         @brief Send content to master

         @param request Request identity
         @param content Requested content
        '''
        payload = {'request': request, 'content': content}
        payload_raw = self.DumpJson(payload)
        if self.__print_payload_info:
            print 'PAYLOAD SLAVE -> MASTER: {0}, {1}'.format(payload['request'], len(payload_raw))
        self.SendLengthFrontMsg(len(payload_raw))
        self.SendLarge(payload_raw, self.__max_send_size)

    def SendLengthFrontMsg(self, length):
        '''
         @brief Send length message of post incoming message.

         @param length Length of post incoming message.
        '''
        content = {'length': length}
        payload = {'request': 'response_size', 'content': content}
        payload_raw = self.DumpJson(payload)
        self.SendLarge(payload_raw, self.__max_send_size)
        self.AssertAck()

    def ReceiveRequest(self, request, content):
        '''
         @brief Handle request from master

         @param request Request from master
         @param content Additional content

         @return stop True/False on stop receiving.
        '''
        stop = False
        if request == 'setNewFrame':
            self.SendPayload(request, '')
            self.__subclass.SetProcessNewFrameFlag()
        elif request == 'getFrame':
            frame_content, error    = self.__subclass.GetFramePayload()
            content                 = self.GetContentRequestFrame(frame_content, error)
            self.SendPayload(request, content)
            if content['valid']:
                self.__subclass.SetStoreDBFlag() # Store frames to database after sending them to master.
        elif request == 'getOriginalFrame':
            sl_filename = None
            if 'sl_filename' in content:
                sl_filename = content['sl_filename']
            filename = content['filename']
            frame_content, error    = self.__subclass.GetOriginalFramePayload(filename, sl_filename)
            content                 = self.GetContentRequestOriginalFrame(frame_content, error)
            self.SendPayload(request, content)
        elif request == 'tradeFrame':
            filename = content['filename']
            frame_content, error    = self.__subclass.GetOriginalFramePayload(filename)
            content, traded_frame   = self.GetRequestTradeFrame(frame_content, content, error)
            self.__subclass.SetNewTradedFrame(traded_frame)
            self.SendPayload(request, content)
        elif request == 'getPointList':
            frame_content, error    = self.__subclass.GetFramePayload()
            content                 = self.GetContentRequestPointList(frame_content, error)
            self.SendPayload(request, content)
            if content['valid']:
                self.__subclass.SetStoreDBFlag() # Store frames to database after sending them to master.
        elif request == 'setTimestamp':
            self.SendPayload(request, '')
            self.__subclass.SetTimestamp(self.GetContentRequestSetTimestamp(content))
        elif request == 'calibrateCV':
            calibrate_stereopsis_session            = content['calibrate_stereopsis_session']
            calibrate_blob_scale_detector_session   = content['calibrate_blob_scale_detector_session']
            self.__subclass.CalibrateCV(calibrate_stereopsis_session, calibrate_blob_scale_detector_session)
            self.SendPayload(request, '')
        elif request == 'slaveReady':
            content = self.GetContentRequestSlaveReady(self.__subclass.GetSlaveReady())
            self.SendPayload(request, content)
        elif request == 'sendFlagToSlave':
            flag = self.GetContentSendFlagToSlave(content)
            self.__subclass.SetFlagFromMaster(flag)
            self.SendPayload(request, '')
        elif request == 'stop':
            self.SendPayload(request, '')
            self.ForceTermination()
        elif request == 'disconnect':
            self.SendPayload(request, '')
            self.Disconnect()
            stop = True
        elif request == 'restart':
            self.SendPayload(request, '')
            self.Disconnect()
            stop = True
            RunThread(self.RestartRequest, daemon=False)
        elif request == 'restartPtGrey':
            self.__subclass.RestartCamera()
            self.SendPayload(request, '')
        elif request == 'error':
            self.SendPayload(request, '')
        return stop

    def RestartRequest(self):
        '''
         @brief Initiated by a restart request from master
        '''
        time.sleep(0.1)
        os.execv(sys.executable, ['python'] + sys.argv)

    def ForceTermination(self):
        '''
         @brief Force termination
        '''
        self.__terminate = True

    def GetTerminate(self):
        '''
         @brief Get Terminate bool.

         @return True/False - True = terminate, False = don't terminate
        '''
        return self.__terminate

    def __del__(self):
        '''DESTRUCTOR'''
        self.Disconnect()