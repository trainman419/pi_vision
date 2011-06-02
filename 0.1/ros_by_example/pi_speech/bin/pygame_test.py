#!/usr/bin/env python

import pygame
import os

pygame.init()

pygame.mixer.pre_init(frequency=16000, size=-16, channels=2, buffer=4000)
pygame.mixer.init(frequency=16000, size=-16, channels=2, buffer=4000)

pickUpSound = pygame.mixer.Sound('/home/patrick/tmp/test.wav')
pickUpSound.set_volume(1.0)
pickUpSound.play()




