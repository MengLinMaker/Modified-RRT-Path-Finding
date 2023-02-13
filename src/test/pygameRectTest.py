import pygame


if __name__ == '__main__':

    size = [10, 20]
    coor = [10, 20]
    bottomLeft = [coor[0] - 0.5 * size[0], coor[1] - 0.5 * size[1]]
    obs = pygame.Rect(bottomLeft, size)

    print(obs.bottomleft == (5, 30))
    print(obs.topleft == (5, 10))
    print(obs.bottomright == (15, 30))
    print(obs.topright == (15, 10))