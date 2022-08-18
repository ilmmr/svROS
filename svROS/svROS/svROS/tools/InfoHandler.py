import inspect

# Class color to use as reference for print handling
# Worth-Mention https://stackoverflow.com/a/17303428
class color:
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    ORANGE = '\033[38;5;208m'
    DARKCYAN = '\033[36m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

    def color(c, text):
        items_to_dict = dict(globals()[__class__.__name__].__dict__.items())
        try:
            config = items_to_dict[c.upper()].strip()
        except Exception as error:
            raise
        return f'{config}{text}{color.END}'

    def bold(function):
        return color.color("BOLD", function)

# Class svROS for output handling
class svException(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return f'[svROS] {color.color("BOLD", color.color("RED", "ERROR:"))} {self.message}'
    
# Class svROS for output handling
class svWarning(object):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return f'[svROS] {color.color("BOLD", color.color("YELLOW", "WARNING:"))} {self.message}'

# Class svROS for output handling
class svInfo(object):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return f'[svROS] {color.color("BOLD", color.color("BLUE", "INFO:"))} {self.message}'