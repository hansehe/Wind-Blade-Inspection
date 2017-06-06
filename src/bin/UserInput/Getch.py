'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master"s Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master"s Thesis - CV (Computer Vision)
'''

############# EXTERNAL LIB ################
'''
 @brief Gets a single character from standard input.  
    Does not echo to the screen.
    Taken from http://code.activestate.com/recipes/134892/
'''
class Getch:
    def __init__(self):
        '''CONSTRUCTOR'''
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        '''
         @brief Returns pressed key char.
          None if no pressed key is waiting to be read.

         @return ch
        '''
        return self.impl()

class _GetchUnix:
    def __init__(self):
        '''CONSTRUCTOR'''
        import tty, sys, termios
        from select import select
        self.__tty      = tty
        self.__sys      = sys
        self.__termios  = termios
        self.__select   = select

    def __call__(self, timeout=0.0):
        '''
         @brief Returns pressed key char.
          None if no pressed key is waiting to be read.

         @return ch
        '''
        ch = None
        fd = self.__sys.stdin.fileno()
        old_settings = self.__termios.tcgetattr(fd)
        try:
            self.__tty.setraw(self.__sys.stdin.fileno())
            rlist, _, _ = self.__select([self.__sys.stdin], [], [], timeout)
            if rlist:
                ch = self.__sys.stdin.read(1)
                #self.__sys.stdin.flush()
        finally:
            self.__termios.tcsetattr(fd, self.__termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        '''CONSTRUCTOR'''
        import msvcrt
        self.__msvcrt = msvcrt

    def __call__(self):
        '''
         @brief Returns pressed key char.
          None if no pressed key is waiting to be read.

         @return ch
        '''
        ch = None
        if self.__msvcrt.kbhit(): 
            ch = self.__msvcrt.getch()
        return ch

#############################################