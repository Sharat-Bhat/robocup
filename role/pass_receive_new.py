import rospy,sys
import enum
import composite_behavior
import behavior
import math
import time
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from utils.functions import *
from tactics import Goalie
import multiprocessing
import threading
#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)

import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=True)

id_passer = 1
id_receiver = 2

class PassReceive(behavior.Behavior):

    passer_kub = kubs.kubs(id_passer,state,pub)
    class State(enum.Enum):

        # we're aligning with the planned direction
        waiting = 1

        # the ball's been kicked and we're adjusting based on where the ball is
        # moving
        adjusting = 2
        receiving = 3
        received = 4

    def __init__(self):
        super(PassReceive, self).__init__()
        self.ball_kicked = False

        for substrate in PassReceive.state:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, 
            PassReceive.State.waiting, lambda: True, "wait for passer to reach ball")
        self.add_transition(PassReceive.State.waiting, 
            PassReceive.State.receiving, lambda: dist(self.kub.get_pos(),self.kub.state.ballPos)<1.5*BOT_RADIUS, "time to receive")
        self.add_transition(PassReceive.State.waiting,
            PassReceive.State.adjusting, lambda: passer_kub.has_ball(), "adjust orientation and position")
        self.add_transition(PassReceive.State.adjusting,
            PassReceive.State.receiving, lambda: dist(self.kub.get_pos(),self.kub.state.ballPos)<1.5*BOT_RADIUS, "ready to receive")
        self.add_transition(PassReceive.State.receiving,
            PassReceive.State.received, lambda: self.kub.has_ball(), "got the ball")

        self.add_transition(PassReceive.State.waiting,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
        self.add_transition(PassReceive.State.adjusting,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')
        self.add_transition(PassReceive.State.receiving,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

    def add_kub(self, kub):
        self.kub = kub

    def execute_waiting(self):
        print("ball ke intazaar mein")
        pass

    def execute_adjusting(self):
        print("position set kar raha hoon")
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)
            theta=angle_diff(self.kub.get_pos(),self.kub.state.ballPos)
            g_fsm = GoToPoint.GoToPoint()       
            g_fsm.add_point(self.kub.get_pos(),theta)               
            g_fsm.add_kub(self.kub)
            print('in receiver executing go to point')
            g_fsm.spin()

    def execute_receiving(self):
        print("ball milne wala hai")
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)
            ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
            ball_angle = ball_vel.angle()
            velocity_magnitude = MAX_BOT_SPEED/(5*BOT_RADIUS)
            face_angle = Vector2D().normalizeAngle(ball_angle + math.pi)
            vx = velocity_magnitude*math.cos(face_angle)
            vy = velocity_magnitude*math.sin(face_angle)
            self.kub.move(vx, vy)
            self.kub.dribble(True)
            self.kub.execute()

    def execute_received(self):
        print("ball mil gaya")
        state = None
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)
        print("enter received")
        self.kub.dribble(False)
        self.kub.execute()
        state = None
        id_receiver, id_passer = id_passer, id_receiver
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("exception found")        
        if state:
            state=state.stateB
            self.kub.update_state(state)

    def check_failure(self):
        THRESH = 1.5 * BOT_RADIUS
        ball_pos = Vector2D(self.kub.state.ballPos)
        kub_pos = Vector2D(self.kub.get_pos.x, self.kub.get_pos.y)
        pass_line = self.pass_line
        ball_line_distance = pass_line.distance_from_point(ball_pos)
        kick_angle = pass_line.angle
        bot_ball_angle = kub_pos.angle(ball_pos)

        opp_kick_angle = Vector2D().normalizeAngle(kick_angle + math.pi)
        diff_angle = Vector2D().normalizeAngle(opp_kick_angle - bot_ball_angle)
        if fabs(diff_angle) < math.pi / 2 and ball_line_distance < THRESH:
            return False
        else:
            return True


def passer(id_self,id_receiver, state,pub):
    kub1 = kubs.kubs(id_self,state,pub)
    print("aaaaaaaaaaaaaaaaaaaa")
    print(id_self)
    kub1.update_state(state)
    target = kubs.kubs(id_receiver,state,pub)
    target.update_state(state)
    #print(kub1.kubs_id)
    g_fsm = KickToPoint.KickToPoint(target.get_pos())
    # g_fsm = GoToPoint.GoToPoint()
    g_fsm.add_kub(kub1)
    # g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
    #g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
    g_fsm.as_graphviz()
    g_fsm.write_diagram_png()
    print("bbb")
    #print('something before spin')
    g_fsm.spin()

def receiver(id_self,id_passer,state,pub):
    kub2 = kubs.kubs(id_self,state,pub)
    kub2.update_state(state)
    print(id_self)
    #print(kub1.kubs_id)
    f_fsm = PassReceive()
    # g_fsm = GoToPoint.GoToPoint()
    f_fsm.add_kub(kub2)
    # g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
    #f_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
    f_fsm.add_theta(theta=normalize_angle(atan2(target.y - state.ballPos.y,target.x - state.ballPos.x)))
    #print('something before spin')
    f_fsm.spin()
    # 
def passer_main(process_id):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node' + str(process_id),anonymous=False)

    while True:
        state = None
        # state=shared.get('state')
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
            state = getState(state)
        except rospy.ServiceException, e:
            print("chutiya")        
        if state:
            #print('lasknfcjscnajnstate',state.stateB.homePos)
            #p2 = multiprocessing.Process(target=receiver, args=(2,state.stateB, )) 
            print("kyun")
            passer(id_passer, id_receiver,state.stateB,pub)
            #p2.start()
            #p1.join()
            #p2.join()
           # print('chal ja')
            # break
        #rospy.spin()   

def receiver_main(process_id):
    pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
    rospy.init_node('node' + str(process_id),anonymous=False)

    while True:
        state = None
        # state=shared.get('state')
        rospy.wait_for_service('bsServer',)
        getState = rospy.ServiceProxy('bsServer',bsServer)
        try:
                state = getState(state)
        except rospy.ServiceException, e:
                print("chutiya")        
        if state:
                #print('lasknfcjscnajnstate',state.stateB.homePos)
                #p2 = multiprocessing.Process(target=receiver, args=(2,state.stateB, )) 
                print("process 2")
                receiver(id_receiver,state.stateB,pub)
                #p2.start()
                #p1.join()
                #p2.join()
               # print('chal ja')
                # break
        #rospy.spin()   


#print str(kub.kubs_id) + str('***********')
p1 = multiprocessing.Process(target=passer_main, args=(1,))
p2 = multiprocessing.Process(target=receiver_main, args=(2,))
p1.start()
p2.start()
p1.join()
p2.join()



# import behavior
# import enum
# import math
# from utils.config import *
# from utils.function import *
# import time
# from role import GoToPoint, GoToBall, KickToPoint
# import os
# import rospy
# import memcache
# shared = memcache.Client(['127.0.0.1:11211'],debug=False)

# def distance(x1, y1, x2, y2):
#     return sqrt((x1 - x2)**2 + (y1 - y2)**2)



#     # returns True if we're facing the right direction and in the right
#     # position and steady
#     def errors_below_thresholds(self):
#         if not isinstance(self.receive_point, Vector2D):
#             return False

#         return (
#             abs(self.angle_diff) < PassReceive.FaceAngleErrorThreshold and
#             abs(self.x_diff) < PassReceive.PositionXErrorThreshold and
#             abs(self.y_diff) < PassReceive.PositionYErrorThreshold)

#     def is_steady(self):
#         kub_vel = self.kub.get_vel()
#         return kub_vel['magnitude'] < PassReceive.SteadyMaxVel and self.kub.vw < PassReceive.SteadyMaxAngleVel

#     def add_kub(self,kub):
#         self.kub = kub

#     def is_ball_near(self):
#         kub_pos = Vector2D(self.kub.get_pos())
#         ball_pos = Vector2D(self.kub.state.ballPos)
#         print("Distance from ball ",ball_pos.dist(kub_pos))
#         return ball_pos.dist(kub_pos) < 5*BOT_RADIUS

#     def recalculate(self):
#         # can't do squat if we don't know what we're supposed to do
#         if not isinstance(self.receive_point, Vector2D) or self.kub == None:
#             return

#         kub_pos = Vector2D(self.kub.get_pos())
#         ball_pos = Vector2D(self.kub.state.ballPos)
#         ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
#         ball_angle = ball_vel.angle()
#         if self.ball_kicked:
#             # when the ball's in motion, the line is based on the ball's
#             # velocity
#             self.pass_line = Line(point1=ball_pos, angle=ball_angle)
#         else:
#             # if the ball hasn't been kicked yet, we assume it's going to go
#             # through the receive point
#             self.pass_line = Line(point1=ball_pos, point2=self.receive_point)


#         if self.ball_kicked:
#             actual_receive_point = self.pass_line.nearest_point(kub_pos)
#         else:
#             actual_receive_point = self.receive_point

#         move_angle = self.pass_line.angle + math.pi
#         self.angle_diff = abs(move_angle)
#         self.x_diff = actual_receive_point.x - self.kub_pos.x
#         self.y_diff = actual_receive_point.y - self.kub_pos.y
        
#     def on_exit_start(self):
#         # reset
#         self.ball_kicked = False

#     def execute_waiting(self):
#         print("execute waiting")
#         pass

#     def execute_adjusting(self):
#         print("execute adjusting")
#         if not isinstance(self.receive_point, Vector2D) or self.kub == None:
#             return

#         kub_pos = Vector2D(self.kub.get_pos())
#         ball_pos = Vector2D(self.kub.state.ballPos)
#         ball_vel = Vector2D(self.kub.state.ballVel.x, self.kub.state.ballVel.y)
#         ball_angle = ball_vel.angle()
        
#         # when the ball's in motion, the line is based on the ball's
#         # velocity
#         self.pass_line = Line(point1=ball_pos, angle=ball_angle)
        
#         actual_receive_point = self.pass_line.nearest_point(kub_pos)
#         move_angle = self.pass_line.angle + math.pi
#         self.angle_diff = abs(move_angle)
#         self.x_diff = actual_receive_point.x - self.kub_pos.x
#         self.y_diff = actual_receive_point.y - self.kub_pos.y
#         self.move = GoToPoint.GoToPoint()
#         state = shared.get('state')
#         self.kub.update_state(state)
#         self.move.add_kub(self.kub)
#         print(move_angle)
#         self.move.add_point(point=actual_receive_point, orient=move_angle)
#         self.add_subbehavior(self.move, 'move')

#     def execute_aligning(self):
#         print("execute aligning")
#         self.recalculate()
#         if isinstance(self._target_pos,Vector2D):
#             move_angle = self.pass_line.angle + math.pi
#             self.move = GoToPoint.GoToPoint()
#             state = shared.get('state')
#             self.kub.update_state(state)
#             self.move.add_kub(self.kub)
#             print(move_angle)
#             self.move.add_point(point=self._target_pos, orient=move_angle)
#             self.add_subbehavior(self.move, 'move')

#     def execute_aligned(self):
#         print("execute aligning")
#         state = shared.get('state')
#         self.kub.update_state(state)
#         point = self.kub.get_pos()
#         self.face = KickToPoint.KickToPoint(self.receive_point)
#         self.face.add_kub(self.kub)
#         self.face.add_point(point)
#         self.add_subbehavior(self.face, 'face')

#     def execute_receiving(self):
#         state = shared.get('state')
#         velocity_magnitude = MAX_BOT_SPEED/(5*BOT_RADIUS)
#         kub_angle = self.pass_line.angle
#         vx = velocity_magnitude*math.cos(kub_angle)
#         vy = velocity_magnitude*math.sin(kub_angle)
#         self.kub.move(vx, vy)
#         self.kub.dribble(True)
#         self.kub.execute()
