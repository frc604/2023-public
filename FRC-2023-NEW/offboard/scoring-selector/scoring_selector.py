import argparse
import pygame

from nt_helper import NTHelper


def run(server):
    # / - + for grid
    # 1-9 for node
    # 0 to confirm

    nt = NTHelper(server)

    pygame.init()
    pygame.display.set_caption("2023 Scoring Selector")
    screen = pygame.display.set_mode([900, 300])

    # Currently selected row and cols
    selected_row = 0  # 0-8 left to right
    selected_col = 0  # 0-2 low to high

    square_size = 100
    grid_size = (3, 3)
    grid_color = (0, 0, 0)
    grid_space = 0
    grid_outline_width = 4
    square_outline_width = 2
    square_highlight_width = 10
    last_highlighted = None

    red = (255, 0, 0)
    green = (0, 200, 0)
    blue = (0, 174, 255)
    purple = (150, 0, 255)
    yellow = (255, 200, 0)
    black = (0, 0, 0)
    gray = (128, 128, 128)

    background_color = (255, 255, 255)
    font = pygame.freetype.Font(None, 36)

    def draw_grids():
        # Draw the left grid
        for row in range(grid_size[0]):
            for col in range(grid_size[1]):
                square_rect = pygame.Rect(
                    col * square_size, row * square_size, square_size, square_size
                )
                if square_rect[1] == 0:  # bottom row (hybrid)
                    pygame.draw.rect(screen, gray, square_rect)
                elif square_rect[1] == 100:  # middle row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)
                elif square_rect[1] == 200:  # top row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)

                pygame.draw.rect(screen, grid_color, square_rect, grid_outline_width)

        # Draw the middle grid
        for row in range(grid_size[0]):
            for col in range(grid_size[1]):
                square_rect = pygame.Rect(
                    (col + grid_size[1] + grid_space) * square_size,
                    row * square_size,
                    square_size,
                    square_size,
                )
                if square_rect[1] == 0:  # bottom row (hybrid)
                    pygame.draw.rect(screen, gray, square_rect)
                elif square_rect[1] == 100:  # middle row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)
                elif square_rect[1] == 200:  # top row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)

                pygame.draw.rect(screen, grid_color, square_rect, grid_outline_width)

        # Draw the right grid
        for row in range(grid_size[0]):
            for col in range(grid_size[1]):
                square_rect = pygame.Rect(
                    (col + grid_size[1] * 2 + grid_space * 2) * square_size,
                    row * square_size,
                    square_size,
                    square_size,
                )
                if square_rect[1] == 0:  # bottom row (hybrid)
                    pygame.draw.rect(screen, gray, square_rect)
                elif square_rect[1] == 100:  # middle row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)
                elif square_rect[1] == 200:  # top row
                    if (
                        square_rect[0] == 0
                        or square_rect[0] == 200
                        or square_rect[0] == 300
                        or square_rect[0] == 500
                        or square_rect[0] == 600
                        or square_rect[0] == 800
                    ):  # cones
                        pygame.draw.rect(screen, yellow, square_rect)
                    elif (
                        square_rect[0] == 100
                        or square_rect[0] == 400
                        or square_rect[0] == 700
                    ):  # cones
                        pygame.draw.rect(screen, purple, square_rect)

                pygame.draw.rect(screen, grid_color, square_rect, grid_outline_width)

    def draw_nums():
        # row 1
        for i in range(0, 9):
            font.render_to(
                screen,
                (square_size / 2 - 10 + i * 100, square_size / 2 - 10),
                str(1 + i),
                black,
            )

        # row 2
        for i in range(0, 9):
            font.render_to(
                screen,
                (square_size / 2 - 10 + i * 100, square_size / 2 - 10 + 100),
                str(1 + i),
                black,
            )

        # row 3
        for i in range(0, 9):
            font.render_to(
                screen,
                (square_size / 2 - 10 + i * 100, square_size / 2 - 10 + 200),
                str(1 + i),
                black,
            )

    def grid_node_id_to_row_col(grid_id, node_id):
        row = node_id // 3
        col = node_id % 3 + 3 * grid_id
        return row, col

    def row_col_to_grid_node_id(row, col):
        grid_id = col // 3
        node_id = col % 3 + 3 * row
        return grid_id, node_id

    def create_squares():
        squares = []
        for i in range(3):  # row
            row_squares = []
            for j in range(9):  # col
                row_squares.append(
                    pygame.Rect(
                        j * square_size,
                        i * square_size,
                        square_size,
                        square_size,
                    )
                )
            squares.append(row_squares)
        return squares

    squares = create_squares()

    mapping = {
        pygame.K_q: (0, 0),
        pygame.K_w: (0, 1),
        pygame.K_e: (0, 2),
        pygame.K_r: (0, 3),
        pygame.K_t: (0, 4),
        pygame.K_y: (0, 5),
        pygame.K_u: (0, 6),
        pygame.K_i: (0, 7),
        pygame.K_o: (0, 8),
        pygame.K_a: (1, 0),
        pygame.K_s: (1, 1),
        pygame.K_d: (1, 2),
        pygame.K_f: (1, 3),
        pygame.K_g: (1, 4),
        pygame.K_h: (1, 5),
        pygame.K_j: (1, 6),
        pygame.K_k: (1, 7),
        pygame.K_l: (1, 8),
        pygame.K_z: (2, 0),
        pygame.K_x: (2, 1),
        pygame.K_c: (2, 2),
        pygame.K_v: (2, 3),
        pygame.K_b: (2, 4),
        pygame.K_n: (2, 5),
        pygame.K_m: (2, 6),
        pygame.K_COMMA: (2, 7),
        pygame.K_PERIOD: (2, 8),
    }

    # Run until the user asks to quit
    running = True
    while running:

        for event in pygame.event.get():
            # Handle quitting
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if mapping.get(event.key) is not None:
                    selected_row, selected_col = mapping.get(event.key)

                if event.key in {pygame.K_0, pygame.K_KP0}:
                    # Send scoring location to robot
                    grid_id, node_id = row_col_to_grid_node_id(
                        selected_row, selected_col
                    )
                    nt.set_scoring_location(grid_id, node_id)

        # Last selected location
        # We read this from NetworkTables instead of storing it locally
        # in order to confirm what the robot sees.
        loc = nt.get_scoring_location()

        # Draw from a blank canvas
        screen.fill(background_color)
        draw_grids()
        draw_nums()

        # Draw all node locations, colored by selected.
        for r, row_nodes in enumerate(squares):
            for c, square in enumerate(row_nodes):
                if loc is not None and grid_node_id_to_row_col(loc[0], loc[1]) == (
                    r,
                    c,
                ):
                    # Draw last selected takes priority over currently selected
                    pygame.draw.rect(screen, red, square, square_highlight_width)
                elif r == selected_row and c == selected_col:
                    pygame.draw.rect(screen, green, square, square_highlight_width)
                else:
                    pygame.draw.rect(screen, black, square, square_outline_width)

        # Flip the display
        pygame.display.flip()

    # Done! Time to quit.
    pygame.quit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--local", action="store_true")
    args = parser.parse_args()
    run("localhost" if args.local else "10.6.4.2")
