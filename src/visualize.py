from PIL import Image

def visualize_path(grid, path, scale=20):
    """
    Prints an ASCII-based visualization of the path on the console
    and also outputs:
      1) A PNG (final state).
      2) A GIF (all states animated).

    Modifications:
      - Uses color (RGB). 
      - Robot shown in red. 
      - Goal shown in green. 
      - Each frame only shows the robot in its current cell 
        (previous location is reverted to normal).
      - The GIF reflects step-by-step movement with no robot trail.

    :param grid:   2D list of single-character strings (map)
                   where:
                     '#' = obstacle
                     'G' = goal
                     'S' = start
                     '.' = free space
    :param path:   List of (row, col) positions (solution path)
    :param scale:  Integer scale factor for upsampling images
    """

    # 1) ASCII Visualization (same as before, but optional).
    #    This still marks a trailing path as 'R' so you can see the route in text.
    #    But for the actual GIF/PNG images, we show only the current robot position.
    grid_copy = [row[:] for row in grid]  # copy the 2D array of characters
    for (r, c) in path:
        if grid_copy[r][c] != '#':
            grid_copy[r][c] = 'R'

    print("\n--- PATH VISUALIZATION (ASCII) ---")
    for row in grid_copy:
        print("".join(row))
    print("----------------------------------\n")

    # --------------------
    # 2) Color Utility
    # --------------------
    def char_to_color(ch):
        """
        Convert a map character to an (R,G,B) color. 
        You can customize these as you like.
        """
        if ch == '#':   # obstacle
            return (0, 0, 0)         # black
        elif ch == 'G': # goal
            return (0, 255, 0)       # green
        elif ch == 'S': # start
            return (200, 200, 200)   # light gray
        elif ch == '.': # free space
            return (255, 255, 255)   # white
        # If we see 'R' or anything else, default to white
        return (255, 255, 255)

    # --------------------
    # 3) Create an image from the grid, 
    #    but optionally overlay the robot in a given (r, c).
    # --------------------
    def grid_to_image(current_grid, robot_pos=None):
        rows = len(current_grid)
        cols = len(current_grid[0]) if rows > 0 else 0

        # Create a 1:1 color image (RGB)
        base_img = Image.new('RGB', (cols, rows))
        pixels = base_img.load()

        # Fill each cell from the grid's color
        for r in range(rows):
            for c in range(cols):
                color = char_to_color(current_grid[r][c])
                pixels[c, r] = color

        # If we have a robot position, draw it in red 
        # (overriding whatever is in that cell).
        if robot_pos is not None:
            rr, cc = robot_pos
            # Safety check for bounds
            if 0 <= rr < rows and 0 <= cc < cols:
                pixels[cc, rr] = (255, 0, 0)  # red

        # Scale up the image so cells are bigger
        big_img = base_img.resize((cols * scale, rows * scale), Image.NEAREST)
        return big_img

    # --------------------
    # 4) Generate frames for the GIF
    #    Each frame: copy the original grid, 
    #    draw the goal in green, place robot in red at current step only.
    # --------------------
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    # For convenience, store the original map in memory
    # We'll place the robot at each step on top of it, 
    # but NOT keep the old position as 'R'.
    # The goal is always 'G' in the grid itself, which is green.
    frames = []
    for t, (r, c) in enumerate(path):
        # The robot is only at (r, c) for this step.
        # We do NOT change the underlying grid permanently,
        # so the robot from the previous step is "removed".
        # This ensures no red trail is left behind.
        img = grid_to_image(grid, (r, c))
        frames.append(img)

    # --------------------
    # 5) Save the final state as PNG
    #    The final state is just the last frame in the sequence
    # --------------------
    final_image = frames[-1]
    final_image.save("path.png")
    print(f"Saved final path image (scaled by {scale}) as path.png")

    # --------------------
    # 6) Save all frames as an animated GIF
    # --------------------
    frames[0].save(
        "path.gif",
        save_all=True,
        append_images=frames[1:],
        duration=300,   # ms per frame
        loop=0          # loop forever
    )
    print(f"Saved path animation (scaled by {scale}) as path.gif")
