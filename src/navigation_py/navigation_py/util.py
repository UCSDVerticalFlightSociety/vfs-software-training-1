import curses
import numpy as np

# screen is 400 x 400
MAX_SCREEN = 400
TARGET_ROW_PIXEL = -1
TARGET_COL_PIXEL = -1
screen = curses.initscr()

# rounding data to utiliy func
# translate floats into screen size user is looking at
# be able to take data given in floats and round that to put onto screen for user to see + understand
# loc_data [x, y] --> need to normalize into actual screen index (row, column)
def get_coords(loc_data: np.ndarray, height, width):

    # normalize
    normalize_x = loc_data[0] / MAX_SCREEN
    normalize_y = loc_data[1] / MAX_SCREEN
    # round
    row = int( round( normalize_x * (height - 1) ) )
    col = int( round( normalize_y * (width - 1) ) )

    return row, col


def update_screen(drone_loc: np.ndarray, target_loc: np.ndarray, screen):
    height, width = screen.getmaxyx()
    screen.clear()

    global TARGET_ROW_PIXEL, TARGET_COL_PIXEL
        # so yk to modify the actual variables rather than reading em locally

    # always update drone but only update target if screen is changed
    # unsure if this is what u wanted??
    if TARGET_ROW_PIXEL == -1 or TARGET_COL_PIXEL == -1:
        TARGET_ROW_PIXEL, TARGET_COL_PIXEL = get_coords(target_loc, height, width)

    drone_row, drone_col = get_coords(drone_loc, height, width)


    # TODO:
    # figure out how to draw on the screen
    # maybe do try except error?? put nested for loop inside of it

    # loop through + if i, j indexes equal drone point / target point
        # then print yay
        # change screen icon at that point accordingly
        # else print some symbol "-"
    # is i and j row and col??
    for i in range(height):
        for j in range(width):
            if i == drone_row and j == drone_col:
                screen.addstr(i, j, '+')
            else:
                screen.addstr(i, j, '-')

    screen.addstr(0, 0, f"Terminal size: {height}x{width}")
    screen.addstr(1, 0, f"Drone: Row = {drone_row}, Column = {drone_col}")
    screen.addstr(1, 0, f"Target: Row = {TARGET_ROW_PIXEL}, Column = {TARGET_COL_PIXEL}")

    screen.refresh()


def main(screen):
    height, width = screen.getmaxyx()

    screen.addstr(0, 0, f"Screen Height: {height}")
    screen.addstr(1, 0, f"Screen Width: {width}")
    screen.refresh()

    if curses.is_term_resized(height, width):
        curses.resize(height, width)
        screen.clear()

    # TODO:
    # some while loop here to cotinue updating the drone
        # should be able to read the data (pos and vel) from ros2 topics
        # call update screen to draw the screen based on drones new position

        # pause func to not overload the screen bc of the constant info
            # i think curses.napms(x) could work or like time.sleep(x)

        # add some sort of key or quit button to get out
            # use screen.getch()
            # wait for user input before exiting

    screen.addstr(0, 0, f"Terminal size: {height}x{width}") # 0,0 coordinates for string

if __name__ == '__main__':
    curses.wrapper(main)