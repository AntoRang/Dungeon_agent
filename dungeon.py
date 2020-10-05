import pygame, sys
import constants
import agents

def get_tile_color(tile_contents):
    tile_color = constants.GOLD
    if tile_contents == ".":
        tile_color = constants.DARKGREY
    if tile_contents == "#":
        tile_color = constants.RED
    if tile_contents == "@":
        tile_color = constants.BLUE
    return tile_color

def draw_map(surface, map_tiles):
    for j, tile in enumerate(map_tiles):
        for i, tile_contents in enumerate(tile):
            # print("{},{}: {}".format(i, j, tile_contents))
            myrect = pygame.Rect(i*constants.BLOCK_WIDTH, j*constants.BLOCK_HEIGHT, constants.BLOCK_WIDTH, constants.BLOCK_HEIGHT)
            pygame.draw.rect(surface, get_tile_color(tile_contents), myrect)

def draw_path(surface, path):
    for coordinate in path:
        myrect = pygame.Rect(coordinate[0]*constants.BLOCK_WIDTH, coordinate[1]*constants.BLOCK_HEIGHT, constants.BLOCK_WIDTH, constants.BLOCK_HEIGHT)
        pygame.draw.rect(surface, get_tile_color("@"), myrect)
        pygame.display.update()

def draw_grid(surface):
    for i in range(constants.NUMBER_OF_BLOCKS_WIDE):
        new_height = round(i * constants.BLOCK_HEIGHT)
        new_width = round(i * constants.BLOCK_WIDTH)
        pygame.draw.line(surface, constants.BLACK, (0, new_height), (constants.SCREEN_WIDTH, new_height), 2)
        pygame.draw.line(surface, constants.BLACK, (new_width, 0), (new_width, constants.SCREEN_HEIGHT), 2)

def game_loop(surface, world_map, path):
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
        draw_map(surface, world_map)
        draw_grid(surface)
        draw_path(surface, path)
        pygame.display.update()

def initialize_game():
    pygame.init()
    surface = pygame.display.set_mode((constants.SCREEN_WIDTH, constants.SCREEN_HEIGHT))
    pygame.display.set_caption(constants.TITLE)
    surface.fill(constants.UGLY_PINK)
    return surface

def read_map(mapfile):
    with open(mapfile, 'r') as f:
        world_map = f.readlines()
    world_map = [line.strip() for line in world_map]
    return (world_map)



def main():
    # world_map = read_map(constants.MAPFILE)
    map = 'map1.txt'
    world_map = read_map(map)
    x, y = 0, 0
    diagrama = agents.GridWithWeights(10, 10)
    diagrama.walls = []
    for line in world_map:
        x = 0
        for idx in line:
            if idx == "G":
                goal = (x, y)
            if idx == "S":
                start = (x, y)
            if idx == "#":
                diagrama.walls.append((x, y))
            x += 1
        y += 1

    came_from_A, cost_so_far_A = agents.a_star_search(diagrama, start, goal)
    (diag, list_coor) = agents.draw_grid(diagrama, path=agents.reconstruct_path(came_from_A, start=start, goal=goal))
    path = agents.convert_path(came_from_A, list_coor)
    a_star = diag.split('\n')[:-1]

    surface = initialize_game()
    game_loop(surface, world_map, path)

if __name__=="__main__":
    main()