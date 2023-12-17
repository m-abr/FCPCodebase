from pathlib import Path
from datetime import datetime
import random
from string import ascii_uppercase

class Logger():
    _folder = None

    def __init__(self, is_enabled:bool, topic:str) -> None:
        self.no_of_entries = 0 
        self.enabled = is_enabled
        self.topic = topic

    def write(self, msg:str, timestamp:bool=True, step:int=None) -> None:
        '''
        Write `msg` to file named `self.topic`

        Parameters
        ----------
        msg : str
            message to be written
        step : int
            simulation step is written before the message to provide additional information
            default is `None` (nothing is written before the message)
        '''
        if not self.enabled: return

        # The log folder is only created if needed
        if Logger._folder is None: 
            rnd = ''.join(random.choices(ascii_uppercase, k=6)) # Useful if multiple processes are running in parallel  
            Logger._folder = "./logs/" + datetime.now().strftime("%Y-%m-%d_%H.%M.%S__") + rnd + "/"
            print("\nLogger Info: see",Logger._folder)
            Path(Logger._folder).mkdir(parents=True, exist_ok=True)

        self.no_of_entries += 1

        with open(Logger._folder + self.topic + ".log", 'a+') as f:
            prefix = ""
            write_step = step is not None
            if timestamp or write_step:
                prefix = "{"
                if timestamp: 
                    prefix += datetime.now().strftime("%a %H:%M:%S")
                    if write_step: prefix += " "
                if write_step:
                    prefix += f'Step:{step}'
                prefix += "} "
            f.write(prefix + msg + "\n")