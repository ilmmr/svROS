import inspect

# Class color to use as reference for print handling
# Worth-Mention https://stackoverflow.com/a/17303428
class color:
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
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
class svROS_Exception(Exception):
    # similair behaviour to raise exception
    @color.bold
    def exception(text='', bold=True):
        if bold:
            exception = color.color("RED", text)
        else:
            exception = color.color("RED", text)
        return exception

# Class svROS for output handling
class svROS_Info:
    # info function     -> predefined color: blue
    def info(text=''):
        info = color.color("RED", text)
        pass
    # warning function  -> predefined color: yellow
    def warn(text=''):
        warn = color.color("YELLOW", text)
        pass
    # set text to bold
    def bold(text=''):
        bold = color.color("BOLD", text)
        pass