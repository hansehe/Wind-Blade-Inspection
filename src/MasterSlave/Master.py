'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import socket, time
from Requests import Requests
from MsgParserRecv.MessageParser import MessageParser

'''
 @brief Master class 

 @param settings_inst (TCP settings)
'''
class Master(MessageParser, Requests):
    def __init__(self, settings_inst):
        '''CONSTRUCTOR'''
        MessageParser.__init__(self)
        Requests.__init__(self, True)
        self.__host                 = settings_inst.GetSettings('master_ip')
        self.__server_port          = settings_inst.GetSettings('port')
        self.__buffer_size          = settings_inst.GetSettings('master_buffer_size')
        self.__max_send_size        = settings_inst.GetSettings('slave_buffer_size')
        self.__timeout              = settings_inst.GetSettings('tcp_timeout')
        self.__print_payload_info   = settings_inst.GetSettings('print_payload_info')
        self.__connected            = False

    def Connect(self):
        '''
         @brief Connect to slave.
        '''
        # Set up the socket connection to the slave
        self.__connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__connection.settimeout(self.__timeout)
        self.__connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Force non-lingering mode of sockets after closing.
        self.__connection.bind((self.__host, self.__server_port))

        # Initiate the connection to the slave
        self.__connection.listen(1)
        self.__slave_conn, self.slave_addr = self.__connection.accept()
        self.__slave_conn.settimeout(self.__timeout)
        self.__connected = True
        
    def Disconnect(self):
        '''
         @brief Disconnect port
        '''
        if self.__connected:
            self.__slave_conn.close()
            self.__connection.close()
            self.__connected = False

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
                payload_raw = self.__slave_conn.recv(self.__buffer_size)
            except:
                self.Disconnect()
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
                offset = self.__slave_conn.send(payload_raw[offset:])
            except:
                self.Disconnect()
                raise

    def RecvResponse(self, request):
        '''
         @brief Receive a response from slave
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
        if not(response == request):
            raise Exception('Response does not match request: Response = ' + response + ', Request = ' + request)
        return content

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
        self.RecvResponse('ack')

    def SendRequest(self, request, content):
        '''
         @brief Send request to slave

         @param request Request identity
         @param content Request content
        '''
        payload = {'request': request, 'content': content}
        payload_raw = self.DumpJson(payload)
        if self.__print_payload_info:
            print 'PAYLOAD MASTER -> SLAVE: {0}, {1}'.format(payload['request'], len(payload_raw))
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

    def RequestFrameProcessingOnSlave(self):
        '''
         @brief Request slave to process new frame
        '''
        request = 'setNewFrame'
        self.SendRequest(request, '')
        content = self.RecvResponse(request)

    def RequestFrame(self):
        '''
         @brief Request frame from slave

         @return frame (numpy array) (OR - None if frame was not possible to get), success Successfull frame request, error Error flag (True/False)
        '''
        request                     = 'getFrame'
        self.SendRequest(request, '')
        content                     = self.RecvResponse(request)
        frame_content, valid, error = self.GetContentRequestFrame(content)
        return frame_content, valid, error

    def RequestOriginalFrame(self, filename, sl_filename=None):
        '''
         @brief Request original stored frame from slave
        '''
        request = 'getOriginalFrame'
        if sl_filename != None:
            content = {'filename': filename, 'sl_filename': sl_filename}
        else:
            content = {'filename': filename}
        self.SendRequest(request, content)
        content = self.RecvResponse(request)
        frame_content, valid, error = self.GetContentRequestOriginalFrame(content)

    def RequestTradeFrame(self, filename, trade_frame):
        '''
         @brief trade frame with slave

         @param filename - requested trade file
         @param trade_frame - frame for trade
        ''' 
        request = 'tradeFrame'
        dtype = str(trade_frame.dtype)
        trade_frame = trade_frame.tolist()
        frame_content = (trade_frame, dtype)
        content = {'filename': filename, 'frame_content': frame_content}
        self.SendRequest(request, content)
        content = self.RecvResponse(request)
        frame_content, valid, error = self.GetRequestTradeFrame(content)
        return frame_content, valid, error

    def RequestPointList(self):
        '''
         @brief Request point list from slave

         @return keypoints, descriptors (list) (OR - None if point list was not possible to get), success Successfull point list request, error Error flag (True/False)
        '''
        request                     = 'getPointList'
        self.SendRequest(request, '')
        content                     = self.RecvResponse(request)
        und_shape, keypoints, descriptors, valid, error = self.GetContentRequestPointList(content)
        return und_shape, keypoints, descriptors, valid, error

    def RequestSetTimestamp(self, timestamp):
        '''
         @brief Request slave to set timestamp

         @param timestamp String
        '''
        request = 'setTimestamp'
        content = self.GetContentRequestSetTimestamp(timestamp)
        self.SendRequest(request, content)
        self.RecvResponse(request)

    def RequestCVCalibration(self, calibrate_stereopsis_session, calibrate_blob_scale_detector_session):
        '''
         @brief Request CV calibration on slave.

         @param calibrate_stereopsis_session (see droneMaster)
         @param calibrate_blob_scale_detector_session (see droneMaster)
        '''
        request = 'calibrateCV'
        content = {'calibrate_stereopsis_session': calibrate_stereopsis_session, 'calibrate_blob_scale_detector_session': calibrate_blob_scale_detector_session}
        self.SendRequest(request, content)
        self.RecvResponse(request)

    def RequestSlaveReady(self):
        '''
         @brief Check if slave is ready. (finished calibrating the camera)

         @return True/False
        '''
        request = 'slaveReady'
        self.SendRequest(request, '')
        content = self.RecvResponse(request)
        ready   = self.GetContentRequestSlaveReady(content)
        return ready

    def SendFlagToSlave(self, flag):
        '''
         @brief Send flag to slave

         @param flag

         @return True/False
        '''
        request = 'sendFlagToSlave'
        self.SendRequest(request, self.GetContentSendFlagToSlave(flag))
        self.RecvResponse(request)

    def RequestStop(self):
        '''
         @brief Request slave stop
        '''
        request = 'stop'
        self.SendRequest(request, '')
        self.RecvResponse(request)

    def RequestDisconnect(self):
        '''
         @brief Request slave stop
        '''
        request = 'disconnect'
        self.SendRequest(request, '')
        self.RecvResponse(request)

    def RequestRestart(self):
        '''
         @brief Request slave restart
        '''
        request = 'restart'
        self.SendRequest(request, '')
        self.RecvResponse(request)
        self.Disconnect()

    def RequestRestartPtGrey(self):
        '''
         @brief Request slave restart PtGrey
        '''
        request = 'restartPtGrey'
        self.SendRequest(request, '')
        self.RecvResponse(request)

    def __del__(self):
        '''DESTRUCTOR'''
        self.Disconnect()