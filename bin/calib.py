#!/usr/bin/env python
import cv
import sys
import pygame
import pygame.locals
import Queue

width = 640
height = 480
red = (255, 0, 0, 255)

color_min = [255, 255, 255]
color_max = [0, 0, 0]

def get_capture():
    cap = cv.CaptureFromCAM(-1)
    cv.SetCaptureProperty(cap, cv.CV_CAP_PROP_FRAME_WIDTH, width)
    cv.SetCaptureProperty(cap, cv.CV_CAP_PROP_FRAME_HEIGHT, height)
    img = cv.QueryFrame(cap)
    img_rgb = cv.CreateMat(height, width, cv.CV_8UC3)
    cv.CvtColor(img, img_rgb, cv.CV_BGR2RGB)
    del(cap)
    return img_rgb.tostring()

def compare_color(color_1, color_2):
    for i in range(0, 3):
        if abs(color_1[i] - color_2[i]) > 10:
            return False
    return True

def add_position(queue, pos):
    for i in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        if pos[0] + i[0] < 0 or pos[0] + i[0] > width - 1:
            continue
        if pos[1] + i[1] < 0 or pos[1] + i[1] > height - 1:
            continue
        queue.put((pos[0] + i[0], pos[1] + i[1]))

def add_color(color):
    for i in range(0, 3):
        color_max[i] = max(color_max[i], color[i])
        color_min[i] = min(color_min[i], color[i])

def fill_color(img, pos, target_color):
    queue = Queue.Queue()
    queue.put(pos)
    while not queue.empty():
        pos = queue.get()
        color = img.get_at(pos)
        if color == red:
            continue
        if not compare_color(target_color, color):
            continue
        img.set_at(pos, red)
        add_color(color)
        add_position(queue, pos)

img = [pygame.image.fromstring(get_capture(), (width, height), "RGB")]
pygame.init()
screen = pygame.display.set_mode((width, height))
while True:
    for event in pygame.event.get():
        if event.type == pygame.locals.QUIT:
            f = open('color.txt', 'w')
            color_min = [str(i) for i in color_min]
            color_max = [str(i) for i in color_max]
            f.write(" ".join(color_min) + " " + " ".join(color_max) + "\n")
            pygame.quit()
            sys.exit()
        if event.type == pygame.locals.MOUSEBUTTONDOWN:
            if event.button == 1:
                img.append(img[-1].copy())
                color = img[-1].get_at(event.pos)
                fill_color(img[-1], event.pos, color)
            elif event.button == 2:
                if len(img) > 1:
                    img.pop()
            elif event.button == 3:
                img = [pygame.image.fromstring(get_capture(), (width, height), "RGB")]
    screen.blit(img[-1], (0, 0))
    pygame.display.update()
