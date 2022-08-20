from cProfile import run
from pkgutil import walk_packages
from time import sleep
from tkinter import CENTER
from turtle import speed
from numpy import angle, arctan, diff
import pygame
import os
import math
import time

#Variable Assignments
pygame.init()
pygame.display.set_caption('drone_wp test')
width, height = 1280, 720
Window = pygame.display.set_mode((width,height))
Win = pygame.Surface((width, height))
fps = 75
SPEED = 5
SPEEDwp = 4
rotspeed = 10
Red = (255,0,0)
mode_x = 282
mode_y = 138


#Font&Text
pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)
Man = myfont.render('Manual Control', True, (0, 0, 0))
Way = myfont.render('Waypoint Control', True, (0, 0, 0))

#Background
Backgroundimg = pygame.image.load(os.path.join('Assets', 'bg.png')).convert()
Background = pygame.transform.scale(Backgroundimg, (width, height)).convert()
    
#Drone Definitions
Drone_W, Drone_H = 75, 75
Droneimg = pygame.image.load(os.path.join('Assets', 'multi-rotor-drone.png')).convert_alpha()
Drone = pygame.transform.rotate(pygame.transform.scale(Droneimg, (Drone_W, Drone_H)), 224).convert_alpha()
dott = pygame.image.load(os.path.join('Assets', 'Basic_red_dot.png')).convert_alpha()
dot = pygame.transform.scale(dott, (20, 20)).convert_alpha()
class starting_screen():
    def player_start(manualmodecollide, waypointmodecollide):
        pygame.draw.rect(Win, (255,255,255), manualmodecollide)
        pygame.draw.rect(Win, (255,255,255), waypointmodecollide)
        Win.blit(Man,(manualmodecollide[0] + 40, manualmodecollide[1] + 45))
        Win.blit(Way,(waypointmodecollide[0] + 20, waypointmodecollide[1] + 45))

#Adding objects to screen
def draw_win(drone, ang, alt, Manual_Contr, WP_Contr, manualmodecollide, waypointmodecollide, wpstart):
    playerpos = [drone.x, drone.y]
    rotated = pygame.transform.rotate(pygame.transform.scale(Drone, (Drone_W + alt, Drone_H + alt)),  ang)
    center = playerpos [0] - int(rotated.get_width() / 2), playerpos [1] - int(rotated.get_height() / 2)
    altfont = pygame.font.SysFont('comicsans', 50)
    alttext = altfont.render("Alt: " + str(alt), 1, (Red))
    Window.blit(Win, (0, 0))
    Win.blit(Background, (0,0))
    starting_screen.player_start(manualmodecollide, waypointmodecollide)
    
    if Manual_Contr == 1:
        Win.blit(Background, (0,0))
        Win.blit(rotated, (center))
        Win.blit(alttext, (1070,650))
    
    elif WP_Contr == 1:
        Win.blit(Background, (0,0))
        Win.blit(rotated, (center))
        Win.blit(alttext, (1070,650))

        if wpstart != 1:
            current_mousepos = pygame.mouse.get_pos()
            Win.blit(dot, (current_mousepos[0] - 10, current_mousepos[1] - 10))
        else:
            pass
    pygame.display.update()
    
#Drone Movements
def move_drone(drone, keys, ang):
    #Left Right Up Down Based on SOHCAHTOA
    if keys[pygame.K_w]: 
            vertical = math.cos(math.radians(ang)) * SPEED
            horizontal = math.sin(math.radians(ang)) * SPEED

            drone.y -= vertical
            drone.x -= horizontal
    if keys[pygame.K_a]:
            vertical = math.cos(math.radians(ang+90)) * SPEED
            horizontal = math.sin(math.radians(ang+90)) * SPEED

            drone.y -= vertical
            drone.x -= horizontal
    if keys[pygame.K_d]:
            vertical = math.cos(math.radians(ang-90)) * SPEED
            horizontal = math.sin(math.radians(ang-90)) * SPEED

            drone.y -= vertical
            drone.x -= horizontal
    if keys[pygame.K_s]:
            vertical = math.cos(math.radians(ang-180)) * SPEED
            horizontal = math.sin(math.radians(ang-180)) * SPEED

            drone.y -= vertical
            drone.x -= horizontal

def WP(event, waypoints, wpstart):
    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3 and wpstart != 1:
        current_mousepos = pygame.mouse.get_pos()
        sleep(1)
        waypoints.append(current_mousepos[0])
        waypoints.append(current_mousepos[1])
        print(waypoints)
    else:
        pass
  
def main():
    #Variables:
    drone = pygame.Rect(65, 72, Drone_W, Drone_H)
    manualmodecollide = pygame.Rect(286, 294, mode_x, mode_y)
    waypointmodecollide = pygame.Rect(718, 294, mode_x, mode_y)
    
    running = True
    clock = pygame.time.Clock()
    ang = -90
    alt = 0

    #Waypoint
    xway = 0
    yway = 1
    waypoints = []
    currentlen = 0
    #Control
    wpstart = 2
    Manual_Contr = 0
    WP_Contr = 0
    wpinit = 1
    wpfinal = 1
    while running:

        clock.tick(fps)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()

        #Call the drone_movements function to move the drone
        keys = pygame.key.get_pressed()
        #Call the draw_win function to draw and blit objects
        draw_win(drone, ang, alt, Manual_Contr, WP_Contr, manualmodecollide, waypointmodecollide, wpstart)
        
#Manual Contr
        if Manual_Contr == 1:
            move_drone(drone, keys, ang)
            
            #Drone Rotation
            if keys[pygame.K_LEFT]:
                ang += 3
                if ang == 360:
                    ang = 0
            if keys[pygame.K_RIGHT]:
                ang -= 3
                if ang == -360:
                    ang = 0
            #Drone Altitude
            if keys[pygame.K_UP] and alt < 500:
                alt += 3
            if keys[pygame.K_DOWN] and alt > 0:
                alt -= 3

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = pygame.mouse.get_pos()
            if manualmodecollide.collidepoint(pos):
                Manual_Contr = 1

        
#Waypoint Contr
        if WP_Contr == 1:
            if wpstart != 1:
                move_drone(drone, keys, ang)
            #Drone Rotation
                if keys[pygame.K_LEFT]:
                    ang += 3
                    if ang == 360:
                        ang = 0
                if keys[pygame.K_RIGHT]:
                    ang -= 3
                    if ang == -360:
                        ang = 0

                #Drone Altitude
                if keys[pygame.K_UP] and alt < 500:
                    alt += 3
                if keys[pygame.K_DOWN] and alt > 0:
                    alt -= 3

            #Waypoints Section
            if wpinit == 1:
                waypoints.append(65)
                waypoints.append(72)
                print(waypoints)
                wpinit = 0
            #Get Waypoints
            
            WP(event, waypoints, wpstart)
            
            #Auto Follow Paths
            if wpstart == 1:
                x_diff = drone.x - waypoints[xway]
                y_diff = drone.y - waypoints[yway]

                Win.blit(dot, (waypoints[xway] - 10, waypoints[yway] - 10))
                
                if y_diff == 0:
                    desired_radian_angle = math.pi/2
                else:
                    desired_radian_angle = math.atan2(x_diff,y_diff)
                
                desired_degree = math.degrees(desired_radian_angle)
                
                if waypoints[yway] > drone.y:
                    desired_radian_angle += math.pi
                
                difference_angle = ang - desired_degree
                if difference_angle >= 180:
                    difference_angle -= 360
                    
                if difference_angle > 0: ang -= min(50, abs(difference_angle))
                else: ang += min(50, abs(difference_angle))

                vertical = math.cos(math.radians(ang)) * (SPEEDwp)
                horizontal = math.sin(math.radians(ang)) * (SPEEDwp)

                drone.y -= vertical
                drone.x -= horizontal
                    
                if x_diff > -10 and x_diff < 10 and y_diff > -10 and y_diff < 10:
                    if len(waypoints)/2 != currentlen:
                        xway += 2
                        yway += 2
                        currentlen += 1
                    if len(waypoints)/2 == currentlen:
                        wpstart = 0

            if wpstart == 0:
                print("Auto Follow Path Ended.")
        
                xway -= len(waypoints)
                yway -= len(waypoints)
                waypoints.clear()
                currentlen = 0
                wpinit = 1
                wpfinal = 1
                drone.x = 65
                drone.y = 72
                ang = -90
                WP_Contr = 0
                wpstart = 2
                sleep(1)

            if keys[pygame.K_RETURN] and len(waypoints) > 2:
                wpstart = 1
                if wpfinal == 1:
                    waypoints.append(65)
                    waypoints.append(72)
                    
                    wpfinal = 0
                

            elif keys[pygame.K_RETURN] and len(waypoints) <= 2:
                print("Not Enough Waypoints!!")

            if keys[pygame.K_ESCAPE]:
                wpstart = 0
            
            
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = pygame.mouse.get_pos()
            if waypointmodecollide.collidepoint(pos):
                WP_Contr = 1
        
#call the main function for the game to init.
if __name__ == "__main__":
    main()