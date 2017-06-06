'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import json

class MessageParser():
    def __init__(self):
        '''CONSTUCTOR'''

        '''Requests'''
        self.__possible_requests = {
            'setNewFrame': self.ParseGeneralPayload,
            'getFrame': self.ParseGeneralPayload,
            'getOriginalFrame': self.ParseGeneralPayload,
            'tradeFrame': self.ParseGeneralPayload,
            'getPointList': self.ParseGeneralPayload,
            'setTimestamp': self.ParseGeneralPayload,
            'calibrateCV': self.ParseGeneralPayload,
            'slaveReady': self.ParseGeneralPayload,
            'sendFlagToSlave': self.ParseGeneralPayload,
            'response_size': self.ParseGeneralPayload,
            'recv_file': self.ParseGeneralPayload,
            'ack': self.ParseGeneralPayload,
            'nack': self.ParseGeneralPayload,
            'stop': self.ParseGeneralPayload,
            'disconnect': self.ParseGeneralPayload,
            'restart': self.ParseGeneralPayload,
            'restartPtGrey': self.ParseGeneralPayload,
            'error': self.ParseGeneralPayload 
        }

    def DumpJson(self, payload):
        '''
         @brief Dump payload to jsonpickle.

         @return payload_raw
        '''
        return json.dumps(payload)

    def LoadJson(self, payload_raw):
        '''
         @brief Load raw payload from jsonpickle.

         @return payload
        '''
        return json.loads(payload_raw)

    def Parse(self, payload_raw):
        '''
         @brief Parse a jsonpickle string

         @param Raw jsonpickle string
        '''
        payload = self.LoadJson(payload_raw)

        if payload['request'] in self.__possible_requests:
            return self.__possible_requests[payload['request']](payload)
        else:
            payload['content'] = 'Request not recognized: ' + payload['request']
            payload['request'] = 'error'
            return ParseError(payload)

    def ParseGeneralPayload(self, payload):
        '''
         @brief Parse a general payload.

         @param payload
        '''
        return (payload['request'], payload['content'])

