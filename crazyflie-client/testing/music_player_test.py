import pygame
import os
import time
# os.environ['SDL_AUDIODRIVER'] = 'pulse'
print('testing')

pygame.mixer.init()
pygame.mixer.music.load("bad_apple.mp3")
pygame.mixer.music.play()
time.sleep(10)
print('ended')