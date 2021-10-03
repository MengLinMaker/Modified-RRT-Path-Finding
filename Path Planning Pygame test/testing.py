import pygame


if __name__ == '__main__':

    size = [0.35, 50]
    coor = [1.2, 0.3]
    upper = [coor[0] + 0.5 * size[0], coor[1] + 0.5 * size[1]]
    obs = pygame.Rect(upper, size)

    print(obs.bottomleft)
    print(obs.topleft)
    print(obs.bottomright)
    print(obs.topright)