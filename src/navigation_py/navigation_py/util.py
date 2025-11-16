# need scrren to be 100 x 100
# rounding data to utiliy func
# translate floats into screen size user is looking at
# be able to take data given in floats and round that to put onto screen for user to see + understand
"""
CIRCLE_PERIOD = 10
TERM_REFRESH_PERIOD = 1 / 60
MEASURED_PUBLISH_PERIOD = 1.5

screen = curses.initscr()
SCREENSIZE = 100, 100
HALFWAY = (SCREENSIZE - 1) / 2
CENTER = np.array([HALFWAY, HALFWAY])

CIRCLERADIUS = int(SCREENSIZE * .4)

"""
import curses

screen = curses.initscr()

def main(screen):
    height, width = screen.getmaxyx()

    screen.addstr(0, 0, f"Screen Height: {height}")
    screen.addstr(1, 0, f"Screen Width: {width}")
    screen.refresh()

    if curses.is_term_resized(height, width):
        curses.resize(height, width)
        screen.clear()
    
    screen.addstr(0, 0, f"Terminal size: {height}x{width}")

    center_x = height // 2
    center_y = width // 2


    # Wait for user input before exiting
    screen.getch()

if __name__ == '__main__':
    curses.wrapper(main)


    # avoid unncessary resizing funcs

# where to put these
curses.noecho()
curses.nocbreak()
screen.keypad(True)
    # respond to special keys idk if u need
    # arrow keys / text editor