# Boids Bird Flocking Simulation
# Life-Like AI navigation and object avoidance
# Generates mesmerising patterns similar to that 
# of flocking birds or schooling fish

# Join Me on my journey to learn!
# This is not wholly my original code, but instead is from GitHub User @meznak 
# who has published their work with the MIT Copyright license
# I have copied and modified their code for the purpose of learning boid flocking concepts

import argparse
import sys
import pygame as pg
from pygame.locals import *
from random import uniform

default_boids = 100
default_geometry = "1280x720"

class bird(pg.sprite.Sprite):
    # each bird is represented by a triangle
    image = pg.Surface((10,10), pg.SRCALPHA)
    pg.draw.polygon(image, pg.Color('white'),[(15,5),(0,2),(0,8)])

    def __init__(self, position, velocity, minSpeed, maxSpeed, maxForce, canWrap):
        super().__init__()

        self.minSpeed = minSpeed
        self.maxSpeed = maxSpeed
        self.maxForce = maxForce

        dimensions = len(position)
        assert(1<dimensions<4), "Invalid spawn position dimensions"
        
        if dimensions == 2:
            self.position = pg.Vector2(position)
            self.acceleration = pg.Vector2(0,0)
            self.velocity = pg.Vector2(velocity)
        else:
            self.position = pg.Vector3(position)
            self.acceleration = pg.Vector3(0,0,0)
            self.velocity = pg.Vector3(velocity)
        
        self.heading = 0.0
        self.rect = self.image.get_rect(center = self.position)

    def update(self, dt, steering):
        self.acceleration = steering * dt

        # limit turning radius
        _, oldHeading = self.velocity.as_polar()
        newVelocity = self.velocity + self.acceleration * dt
        speed, newHeading = newVelocity.as_polar()

        headingDiff = 180 - (180 - newHeading + oldHeading) % 360
        if abs(headingDiff) > self.maxTurn:
            if headingDiff > self.maxTurn:
                newHeading = oldHeading + self.maxTurn
            else:
                newHeading = oldHeading - self.maxTurn
        
        self.velocity.from_polar((speed, newHeading))

        # enforce speed bounds
        speed, self.heading = self.velocity.as_polar()
        if speed < self.minSpeed:
            self.velocity.scale_to_length(self.minSpeed)
        if speed > self.maxSpeed:
            self.velocity.scale_to_length(self.maxSpeed)

        # move each frame
        self.position += self.velocity * dt

        if self.canWrap:
            self.wrap()
        
        # draw
        self.image = pg.transform.rotate(bird.image, -self.heading)

        if self.debug:
            center = pg.Vector2((50,50))

            velocity = pg.Vector2(self.velocity)
            speed = velocity.length()
            velocity += center

            acceleration = pg.Vector2(self.acceleration)
            acceleration += center

            steering = pg.Vector2(steering)
            steering += center

            overlay = pg.Surface((100,100), pg.SRCALPHA)
            overlay.blit(self.image, center - (10,10))

            pg.draw.line(overlay, pg.Color('green'), center, velocity, 3)
            pg.draw.line(overlay, pg.Color('red'), center + (5,0), acceleration + (5,0), 3)
            pg.draw.line(overlay, pg.Color('blue'), center - (5,0), steering - (5,0), 3)
            
            self.image = overlay
            self.rect = overlay.get_rect(center=self.position)
        else:
            self.rect = self.image.get_rect(center=self.position)
    
    def avoid_edge(self):
        left = self.edges[0] - self.position.x
        up = self.edges[1] - self.position.y
        right = self.position.x - self.edges[2]
        down = self.position.y - self.edges[3]

        scale = max(left, up, right, down)

        if scale > 0:
            center = (bird.max_x / 2, bird.max_y / 2)
            steering = pg.Vector2(center)
            steering -= self.position
        else:
            steering = pg.Vector2()
        
        return steering
    
    def wrap(self):
        if self.position.x < 0:
            self.position.x += bird.max_x
        elif self.position.x > bird.max_x:
            self.position.x -= bird.max_x
        
        if self.position.y < 0:
            self.position.y += bird.max_y
        elif self.position.y > bird.max_y:
            self.position.y -= bird.max_y

    @staticmethod
    def set_boundary(edge_distance_pct):
        info = pg.display.Info()
        bird.max_x = info.current_w
        bird.max_y = info.current_h
        margin_w = bird.max_x * edge_distance_pct / 100
        margin_h = bird.max_y * edge_distance_pct / 100
        bird.edges = [margin_w, margin_h, bird.max_x - margin_w, bird.max_y - margin_h]

    def clamp_force(self, force):
        if 0 < force.magnitude() > self.maxForce:
            force.scale_to_length(self.maxForce)

        return force
        
class boid(bird):
    # config variables
    debug = False
    minSpeed = 0.01
    maxSpeed = 0.2
    maxForce = 1
    maxTurn = 5
    perception = 60
    crowding = 15
    canWrap = True
    edge_distance_pct = 5

    def __init__(self):
        boid.set_boundary(boid.edge_distance_pct)

        # random starting position and velocity
        start_position = pg.math.Vector2(uniform(0, boid.max_x), uniform(0, boid.max_y))
        start_velocity = pg.math.Vector2(uniform(-1, 1) * boid.maxSpeed, uniform(-1, 1) * boid.maxSpeed)

        super().__init__(start_position, start_velocity, boid.minSpeed, boid.maxSpeed, boid.maxForce, boid.canWrap)

        self.rect = self.image.get_rect(center=self.position)
        self.debug = boid.debug

    def separation(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            dist = self.position.distance_to(boid.position)
            if dist < self.crowding:
                steering -= boid.position - self.position
        steering = self.clamp_force(steering)
        return steering
    
    def alignment(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.position
        steering /= len(boids)
        steering -= self.velocity
        steering = self.clamp_force(steering)
        return steering / 8

    def cohesion(self, boids):
        steering = pg.Vector2()
        for boid in boids:
            steering += boid.position
        steering /= len(boids)
        steering -= self.position
        steering = self.clamp_force(steering)
        return steering / 100
    
    def update(self, dt, boids):
        steering = pg.Vector2()

        if not self.canWrap:
            steering += self.avoid_edge()

        neighbors = self.get_neighbors(boids)
        if neighbors:
            separation = self.separation(neighbors)
            alignment = self.alignment(neighbors)
            cohesion = self.cohesion(neighbors)

            steering += separation + alignment + cohesion

        super().update(dt, steering)

    def get_neighbors(self, boids):
        neighbors = []
        for boid in boids:
            if boid != self:
                dist = self.position.distance_to(boid.position)
                if dist < self.perception:
                    neighbors.append(boid)

        return neighbors

def update(dt, boids):
    for event in pg.event.get():
        if event.type == QUIT:
            pg.quit()
            sys.exit(0)
        elif event.type == KEYDOWN:
            mods = pg.key.get_mods()
            if event.key == pg.K_q:
                pg.quit()
                sys.exit(0)
            elif event.key == pg.K_UP:
                # adds boids
                add_boids(boids, 10)
            elif event.key == pg.K_DOWN:
                # removes boids
                boids.remove(boids.sprites()[:10])
            elif event.key == pg.K_r:
                # reset the simulation
                num_boids = len(boids)
                boids.empty()
                add_boids(boids, num_boids)
                
    for b in boids:
        b.update(dt, boids)

def draw(screen, background, boids):
    boids.clear(screen, background)
    dirty = boids.draw(screen)
    pg.display.update(dirty)

def main(args):
    pg.init()
    pg.event.set_allowed([pg.QUIT, pg.KEYDOWN, pg.KEYUP])

    fps = 60.0
    fpsClock = pg.time.Clock()

    window_width, window_height = [int(x) for x in args.geometry.split("x")]
    flags = DOUBLEBUF

    screen = pg.display.set_mode((window_width, window_height), flags)
    screen.set_alpha(None)

    background = pg.Surface(screen.get_size()).convert()
    background.fill(pg.Color('black'))

    boids = pg.sprite.RenderUpdates()

    add_boids(boids, args.num_boids)

    dt = 1/fps

    while True:
        update(dt, boids)
        draw(screen, background, boids)
        dt = fpsClock.tick(fps)

def add_boids(boids, num_boids):
    for _ in range(num_boids):
        boids.add(boid())
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Emergent Flocking")
    parser.add_argument('--geometry', metavar='WxH', type=str, 
                        default=default_geometry, help="Geometry of the Window")
    parser.add_argument('--number', dest = 'num_boids', 
                        default=default_boids, help="Number of boids to simulate")
    args = parser.parse_args()
    main(args)

