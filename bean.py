#https://github.com/RLBot/RLBot/wiki/Useful-Game-Values

from glm import *
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

fieldBoxCenter = vec3(0, 0, 1022)
fieldBoxExtent = vec3(4096, 5120, 1022)
g = 650


def mat3FromRot(rot):
    cr = cos(rot.x)
    sr = sin(rot.x)
    cp = cos(rot.y)
    sp = sin(rot.y)
    cy = cos(rot.z)
    sy = sin(rot.z)


    forward = 1*vec3(cp * cy, cp * sy, sp)
    right = vec3(cy*sp*sr-cr*sy, sy*sp*sr+cr*cy, -cp*sr)
    up = vec3(-cr*cy*sp-sr*sy, -cr*sy*sp+sr*cy, cp*cr)

    return mat3(normalize(right), normalize(forward), normalize(up))


class Bean(BaseAgent):
    def initialize_agent(self):
        self.controllerState = None
        self.controllerStateOld = SimpleControllerState()

        self.pos = None
        self.vel = None
        self.rot = None #roll pitch yaw
        self.mat = None
        self.angularVel = None
        self.ballPos = None
        self.ballVel = None
        self.ballPosLocal = None
        self.hitbox = None
        self.onGround = None
        self.onGroundStable = None
        self.hasJumpLeft = None

        self.time = 0
        self.deltaTime = 0
        self.timeSinceLeftGround = 0

    def estimateTimeTo(self, p):
        d = length(self.pos-p)

        #velTowardsTarget = length(self.vel)
        #velTowardsTarget *= 0.5+0.5*max(0, dot(normalize(self.vel), normalize(p-self.pos)))
        return d/length(self.vel)

    def findTargetPoint(self):
        minError = 1000000
        minErrorPos = None

        ball_prediction = self.get_ball_prediction_struct()
        if ball_prediction is not None:
            for i in range(0, ball_prediction.num_slices):
                prediction_slice = ball_prediction.slices[i]
                location = vec3(prediction_slice.physics.location.x, prediction_slice.physics.location.y, prediction_slice.physics.location.z)
                ballT = prediction_slice.game_seconds - self.time

                carT = self.estimateTimeTo(location)
                error = abs(carT-ballT)
                if error < minError:
                    minError = error
                    minErrorPos = location
        return minErrorPos


    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        # INIT
        self.renderer.begin_rendering()
        
        self.ballPos = vec3(packet.game_ball.physics.location.x, packet.game_ball.physics.location.y, packet.game_ball.physics.location.z)
        self.ballVel = vec3(packet.game_ball.physics.velocity.x, packet.game_ball.physics.velocity.y, packet.game_ball.physics.velocity.z)
        
        my_car = packet.game_cars[self.index]
        self.pos = vec3(my_car.physics.location.x, my_car.physics.location.y, my_car.physics.location.z)
        self.vel = vec3(my_car.physics.velocity.x, my_car.physics.velocity.y, my_car.physics.velocity.z)
        self.angularVel = vec3(my_car.physics.angular_velocity.x, my_car.physics.angular_velocity.y, my_car.physics.angular_velocity.z)
        self.rot = vec3(my_car.physics.rotation.roll, my_car.physics.rotation.pitch, my_car.physics.rotation.yaw)
        self.mat = mat3FromRot(self.rot)
        self.hitbox = vec3(my_car.hitbox.width, my_car.hitbox.length, my_car.hitbox.height)
        self.onGround = my_car.has_wheel_contact
        self.hasJumpLeft = not my_car.double_jumped
        
        self.deltaTime = packet.game_info.seconds_elapsed - self.time
        self.time = packet.game_info.seconds_elapsed
        self.ballPosLocal = self.mat * (self.ballPos-self.pos)

        # BOT CODE
        if self.onGround:
            self.timeSinceLeftGround = 0
        else:
            self.timeSinceLeftGround += self.deltaTime

        target = self.ballPos#self.findTargetPoint()
        drawPoint(self.renderer, target)
        
        self.controllerState = SimpleControllerState()

        self.controllerState.throttle = 1.0

        turnPlaneNor = cross(normalize(target - self.pos), self.mat[2])
        self.controllerState.steer = clamp(dot(self.mat[1], turnPlaneNor)*2, -1, 1)
        self.controllerState.boost = True

 
        upwardsVel = dot(self.vel, self.mat[2])
        self.onGroundStable = upwardsVel > 0 and self.onGround

        facingTarget = dot(normalize(self.vel), normalize(target-self.pos))
        velTowardsTarget = 0
        if facingTarget > 0:
            velTowardsTarget = length(self.vel) * pow(facingTarget, 2)
        
        shouldDodgeToGainSpeedGround = velTowardsTarget > 1200 and velTowardsTarget < 2000
        shouldDodgeToGainSpeedAir = velTowardsTarget > 1100 and velTowardsTarget < 2100
        
        if shouldDodgeToGainSpeedGround and self.onGroundStable and abs(dot(self.angularVel, self.mat[2])) < 1:
            self.controllerState.jump = True
            print("initiate jump")

        if shouldDodgeToGainSpeedAir and self.timeSinceLeftGround < 0.1 and self.controllerStateOld.jump:
            self.controllerState.jump = True
            print("hold jump")

        if shouldDodgeToGainSpeedAir and self.timeSinceLeftGround > 0.1 and self.hasJumpLeft and not self.controllerStateOld.jump:
            self.controllerState.jump = True
            print("dodge")
            ballPosLocalPredict = self.mat * (self.ballPos-self.pos)
            self.controllerState.pitch = -1
            self.controllerState.yaw = clamp(dot(self.mat[1], turnPlaneNor)*4, -1, 1)

        self.renderer.end_rendering()

        self.controllerStateOld.__dict__.update(self.controllerState.__dict__) # controllerStateOld = controllerState, copy values
        return self.controllerState


def draw_debug(renderer, car, ball, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(car.physics.location, ball.physics.location, renderer.white())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    renderer.end_rendering()

def drawLine(renderer, a, b):
    renderer.draw_line_3d([a.x, a.y, a.z], [b.x, b.y, b.z], renderer.red())

def drawPoint(renderer, p):
    renderer.draw_rect_3d([p.x, p.y, p.z], 5, 5, True, renderer.red())
