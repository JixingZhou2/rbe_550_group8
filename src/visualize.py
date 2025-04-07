from PIL import Image

def visualize_path(grid, path, box_paths=None, scale=20):
    """
    Visualizes the robot path and box motions.
    Draws:
      - Robot in red
      - Boxes in blue
      - Goal in green
      - Obstacles in black
      - Free space in white

    :param grid:       2D list of characters
    :param path:       List of (r, c) robot positions at each time
    :param box_paths:  List of box position lists, one per box
    :param scale:      Image scale
    """
    grid_copy = [row[:] for row in grid]
    for (r, c) in path:
        if grid_copy[r][c] != '#':
            grid_copy[r][c] = 'R'

    print("\n--- PATH VISUALIZATION (ASCII) ---")
    for row in grid_copy:
        print("".join(row))
    print("----------------------------------\n")

    def char_to_color(ch):
        if ch == '#': return (0, 0, 0)       # obstacle -> black
        elif ch == 'G': return (0, 255, 0)   # goal -> green
        elif ch == 'S': return (200, 200, 200)  # start -> gray
        elif ch == '.': return (255, 255, 255)  # free -> white
        return (255, 255, 255)

    def grid_to_image(current_grid, robot_pos=None, box_pos_list=None):
        rows = len(current_grid)
        cols = len(current_grid[0]) if rows > 0 else 0
        base_img = Image.new('RGB', (cols, rows))
        pixels = base_img.load()

        for r in range(rows):
            for c in range(cols):
                pixels[c, r] = char_to_color(current_grid[r][c])

        if box_pos_list:
            for (br, bc) in box_pos_list:
                if 0 <= br < rows and 0 <= bc < cols:
                    pixels[bc, br] = (0, 0, 255)  # box -> blue

        if robot_pos:
            rr, cc = robot_pos
            if 0 <= rr < rows and 0 <= cc < cols:
                pixels[cc, rr] = (255, 0, 0)  # robot -> red

        return base_img.resize((cols * scale, rows * scale), Image.NEAREST)

    # --- Generate animation frames ---
    frames = []
    num_frames = len(path)
    for t in range(num_frames):
        robot_here = path[t]
        box_positions = []
        if box_paths:
            for b_path in box_paths:
                if t < len(b_path):
                    box_positions.append(b_path[t])
        img = grid_to_image(grid, robot_here, box_positions)
        frames.append(img)

    # Extract final robot and box positions
    final_robot = path[-1]
    final_boxes = []
    if box_paths:
        for b in box_paths:
            if len(b) > 0:
                final_boxes.append(b[-1])

    final_image = grid_to_image(grid, final_robot, final_boxes)
    final_image.save("path.png")

    print(f"Saved final path image (scaled by {scale}) as path.png")

    frames[0].save(
        "path.gif",
        save_all=True,
        append_images=frames[1:],
        duration=300,
        loop=0
    )
    print(f"Saved path animation (scaled by {scale}) as path.gif")
