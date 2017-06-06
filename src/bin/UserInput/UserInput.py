'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master"s Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master"s Thesis - CV (Computer Vision)
'''

import threading
from src.bin.tools import RunThread
from Getch import Getch
from getpass import getpass

'''
 @brief Handle user input

 @param input_settings (USER_INPUT)
'''
class UserInput():
    def __init__(self, input_settings):
        '''CONSTRUCTOR'''
        self.__terminate            = False
        self.__capture_img          = False
        self.__getch_lock           = threading.Lock()
        self.__getch                = Getch() # Get new user input by calling self.__getch()
        self.__action_keys          = {  input_settings.GetSettings('key_terminate'): 'terminate'}
        self.__manual_user_input    = True
        self.__auto                 = input_settings.GetSettings('automatic_mode')
        self.__no_termination       = input_settings.GetSettings('no_termination')

    def GetYesNoFromUser(self, question=''):
        '''
         @brief ask user for a yes / no answer

         @param question (string)

         @return True/False (yes/no)
        '''
        answer      = False
        question    += ' [Y/n]: '
        reply = raw_input(question)
        if reply.upper() == 'Y':
            answer = True
        return answer

    def GetActionKeys(self):
        '''
         @brief Get action keys (dictionary)

         @return action_keys
        '''
        return self.__action_keys

    def GetInput(self):
        '''
         @brief Get user input
            Returns user input as char.
            Returns None if no pressed key was waiting to be read.

         @return char
        '''
        return self.__getch()

    def ForceTermination(self):
        '''
         @brief Force termination
        '''
        self.__terminate = True

    def CheckTerminated(self):
        '''
         @brief Check if user input is terminated

         @return True/False
        '''
        if self.__manual_user_input:
            self.ManualHandleUserInput()
        return self.__terminate

    def StartAutoHandleUserInput(self):
        '''
         @brief Handle auto user input in thread
        '''
        if self.__auto:
            self.__manual_user_input = False
            if not(self.__no_termination):
                RunThread(self.AutoHandleUserInput)

    def PauseUserInput(self):
        '''
         @brief Pause user input thread
        '''
        while self.__getch_lock.locked():
            pass
        self.__getch_lock.acquire()

    def RestartUserInput(self):
        '''
         @brief Restart user input after invoking a pause
        '''
        if self.__getch_lock.locked():
            self.__getch_lock.release()

    def HandleActionKey(self, action_key):
        '''
         @brief Handle action key

         @param action_key
        '''
        ######## HANDLE ACTION KEYS ##########
        if action_key == 'terminate':
            self.ForceTermination()
        ######################################

    def PrintActionKeys(self):
        '''
         @brief Print action key possibilites
        '''
        print '\nPossible action keys ("action_key" -> action):'
        for key in self.__action_keys:
            print '\t"{0}" -> {1}'.format(key, self.__action_keys[key])
        print '..or just enter to continue'

    def AutoHandleUserInput(self, print_keys=False):
        '''
         @brief Handle user input automatically.

         @param print_keys (Default=False)
        '''
        self.__terminate = False
        while not(self.CheckTerminated()):
            with self.__getch_lock:
                ch = self.GetInput()
            if not(ch == None):
                if print_keys:
                    print ch
                if ch in self.__action_keys:
                    action_key = self.__action_keys[ch]
                    self.HandleActionKey(action_key)

    def ManualHandleUserInput(self, print_keys=False):
        '''
         @brief Handle user input manually.

         @param print_keys (Default=False)
        '''
        self.__terminate = False
        self.PrintActionKeys()
        ch = getpass('Type any of the action keys and enter to continue..')
        if print_keys:
            print ch
        if ch in self.__action_keys:
            action_key = self.__action_keys[ch]
            self.HandleActionKey(action_key)
                    

    def __del__(self):
        '''DESTRUCTOR'''
        self.ForceTermination()