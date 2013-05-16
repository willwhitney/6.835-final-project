from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import Leap, sys, Queue
import numpy

firstHistoryFrame = 30
lastHistoryFrame = 40
numPastFrames = lastHistoryFrame - firstHistoryFrame

averageWindow = 5

fingerDecreaseThreshold = 2
fingerIncreaseThreshold = 1.5
radiusDecreaseThreshold = 5

palmVelocityThreshold = 500

controlQueue = Queue.Queue()
pointerQueue = Queue.Queue()

center = [0, 0, 0]
pointerPosition = None
pointerData = (None, False)
angle = [0, 0, 0]
translation = [0, 0, 0]
rotation = [0, 0, 0]

postPointingAngle = None

class GestureController(Leap.Listener):
    def on_init(self, controller):
        self.activeState = 'open'
        self.inactiveState = 'closed'
        self.state = self.inactiveState
        self.translated = False
        self.controlHand = None
        print "Initialized"

    def setControlQueue(self, controlQueue):
        self.controlQueue = controlQueue

    def setPointerQueue(self, pointerQueue):
        self.pointerQueue = pointerQueue

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def average_hands(self, controller, start, stop, id=None):
        hand = {
            'sphere_radius': 0,
            'sphere_center': Leap.Vector(0, 0, 0),
            'numFingers': 0,
            'x': 0,
            'y': 0,
            'z': 0,
            'pitch': 0,
            'roll': 0,
            'yaw': 0
        }
        for i in xrange(start, stop, 1):
            frame = controller.frame(i)
            if len(frame.hands) == 0:
                return None

            if id != None:
                for h in frame.hands:
                    if h.id == id:
                        thisHand = h
            else:
                thisHand = frame.hands[0]

            hand['sphere_radius'] += thisHand.sphere_radius
            hand['sphere_center'] += thisHand.palm_position
            hand['numFingers'] += len(thisHand.fingers)
            hand['x'] += thisHand.palm_position.x 
            hand['y'] += thisHand.palm_position.y
            hand['z'] += thisHand.palm_position.z
            hand['pitch'] += thisHand.direction.pitch * Leap.RAD_TO_DEG
            hand['roll'] += thisHand.palm_normal.roll * Leap.RAD_TO_DEG
            hand['yaw'] += thisHand.direction.yaw * Leap.RAD_TO_DEG
        hand = {
            'id': thisHand.id,
            'sphere_radius': hand['sphere_radius'] / (stop - start),
            'sphere_center': hand['sphere_center'] / (stop - start),
            'numFingers': float(hand['numFingers']) / (stop - start),
            'x': hand['x'] / (stop - start),
            'y': hand['y'] / (stop - start),
            'z': hand['z'] / (stop - start),
            'pitch': hand['pitch'] / (stop - start),
            'roll': hand['roll'] / (stop - start),
            'yaw': hand['yaw'] / (stop - start)
        }
        return hand

    def get_fingers(self, controller, historyDepth=0, handId=None):
        fingers = []
        frame = controller.frame(historyDepth)
        if len(frame.hands) == 0:
            return None

        if handId != None:
            for h in frame.hands:
                if h.id == handId:
                    thisHand = h

        if len(thisHand.fingers) < 0:
            return None
        return thisHand.fingers

    def one_recent_hand(self, controller, count):
        for i in xrange(0, count, 1):
            frame = controller.frame(i)
            if len(frame.hands) != 0:
                return True
        return False

    def all_recent_hands(self, controller, count):
        handFrames = 0
        for i in xrange(0, count, 1):
            frame = controller.frame(i)
            if len(frame.hands) != 0:
                handFrames += 1
        return handFrames > .95 * count

    def set_state(self, state, controller):
        if state == self.state:
            return
        if state == self.activeState:
            self.controlHand = controller.frame().hands[0].id
        else:
            self.controlHand = None
        self.state = state

    def calculate_intersection(self, sphereCenter, sphereRadius, vectorStart, vectorDirection):
        determinant = (vectorDirection.dot(vectorStart - sphereCenter)) ** 2 - \
                ((vectorStart - sphereCenter).dot((vectorStart - sphereCenter)) \
                - sphereRadius**2)

        if determinant < 0:
            return None

        distances = []
        distances.append(-1 * (vectorDirection.dot(vectorStart - sphereCenter)) + determinant ** (0.5))
        distances.append(-1 * (vectorDirection.dot(vectorStart - sphereCenter)) - determinant ** (0.5))

        # distances = filter(lambda x: x >=0, distances)
        if len(distances) <= 0:
            return None
        distance = min(distances, key=lambda x: abs(x))

        intersection = vectorStart + vectorDirection * distance

        direction = (intersection - sphereCenter) / ((intersection - sphereCenter).dot(intersection - sphereCenter)) ** (0.5)
        return direction

    def on_frame(self, controller):
        frame = controller.frame()

        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #       frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))
        # print [frame.hands[i].id for i in range(len(frame.hands))]
        if len(frame.hands) > 0:
            hand = frame.hands[0]
            velocity = (hand.palm_velocity.x ** 2 + hand.palm_velocity.y ** 2 + hand.palm_velocity.z ** 2) **(1./2)
            if velocity < palmVelocityThreshold:
                if self.state == 'open':
                    hand = self.average_hands(controller, 0, averageWindow)
                    oldHand = self.average_hands(controller, firstHistoryFrame, lastHistoryFrame)

                    if hand != None and oldHand != None:
                        if oldHand['numFingers'] - hand['numFingers'] > fingerDecreaseThreshold \
                                and oldHand['sphere_radius'] - hand['sphere_radius'] > radiusDecreaseThreshold:
                            print "Closing Gesture Detected! \n\n"
                            self.set_state('closed', controller)
                            return

                else:
                    hand = self.average_hands(controller, 0, averageWindow)
                    oldHand = self.average_hands(controller, firstHistoryFrame, lastHistoryFrame)

                    if hand != None and oldHand != None:
                        if oldHand['numFingers'] - hand['numFingers'] < -1 * fingerIncreaseThreshold \
                                and oldHand['sphere_radius'] - hand['sphere_radius'] < -1 * radiusDecreaseThreshold:
                            print "Opening Gesture Detected! \n\n"
                            self.set_state('open', controller)
                            return

                if self.state == self.activeState:
                    controlHand = None
                    pointerHand = None
                    for hand in controller.frame().hands:
                        if hand.id == self.controlHand:
                            controlHand = self.average_hands(controller, 0, averageWindow, hand.id)
                            self.lastControlHand = self.average_hands(controller, averageWindow + 1, 2 * averageWindow, hand.id)
                        else:
                            pointerHand = self.average_hands(controller, 0, averageWindow, hand.id)

                    if self.lastControlHand != None and controlHand != None:
                        translation = (
                                controlHand['x'] - self.lastControlHand['x'], 
                                controlHand['y'] - self.lastControlHand['y'], 
                                controlHand['z'] - self.lastControlHand['z']
                        )
                        rotation = (
                                controlHand['pitch'] - self.lastControlHand['pitch'], 
                                controlHand['roll'] - self.lastControlHand['roll'], 
                                controlHand['yaw'] - self.lastControlHand['yaw']
                        )
                        controlQueue.put(translation + rotation)

                    if pointerHand != None and controlHand != None:
                        # lastPointerFinger = self.get_fingers(controller, 1, pointerHand['id'])[0]

                        pointerHandFingers = self.get_fingers(controller, 0, pointerHand['id'])
                        if pointerHandFingers != None and len(pointerHandFingers) > 0:
                            pointerFinger = pointerHandFingers[0]

                            relativePointerPosition = self.calculate_intersection(controlHand['sphere_center'], \
                                    150, pointerFinger.tip_position, pointerFinger.direction)
                            if type(relativePointerPosition) != type(None):
                                pointerData = (relativePointerPosition.to_float_array(), True)
                                pointerQueue.put(pointerData)
                    else:
                        pointerQueue.put((None, False))


        else:
            if not self.one_recent_hand(controller, 10):
                self.set_state(self.inactiveState, controller)

def updatePosition():
    updated = not controlQueue.empty() or not pointerQueue.empty()
    global center
    global angle
    while not controlQueue.empty():
        movement = list(controlQueue.get_nowait())
        translation = [movement[i] / 200 for i in range(3)]
        rotation = [movement[i] / 4 for i in range(3, 6)]

        center = [center[i] + translation[i] for i in range(3)]
        angle = [angle[i] + rotation[i] for i in range(3)]

    global pointerPosition
    global pointerData
    global postPointingAngle
    # global angle
    while not pointerQueue.empty():
        pointerData = pointerQueue.get_nowait()
        if pointerData[0] != None and pointerData[1]:
            pointerPosition = pointerData[0]
            postPointingAngle = None
        if not pointerData[1] and postPointingAngle == None:
            postPointingAngle = angle
    if updated:
        glutPostRedisplay()

 
# This function is called whenever a "Normal" key press is received.
def keyboardFunc( key, x, y ):
    # exit if ENTER or ESC is pressed
    if key == '\r' or key == '\x1b':
        exit()
    elif key == 'r':
        angle[0] += 20
        glutPostRedisplay()
    elif key == 't':
        center[0] += 1
        center[1] += 0
        center[2] += 0
        glutPostRedisplay()
    elif key == 's':
        glTranslatef(-center[0], -center[1], -center[2])
        glScalef(1.5,1.5,1.5)
        center[0] /= 1.5
        center[1] /= 1.5
        center[2] /= 1.5
        glTranslatef(center[0], center[1], center[2])
        glutPostRedisplay()

def drawScene():
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    
    currentlyPointing = pointerData[1]

    # --------------------------- OBJECT ---------------------------
    glPushMatrix()

    # reverseCenter = [-1 * center[i] for i in range(3)]
    glLoadIdentity();
    # Position the camera at [0,0,20], looking at [0,0,0],
    # with [0,1,0] as the up direction.
    gluLookAt(0,0,10,
              0,0,0,
              0,1,0)

    # print center, angle
    # glTranslate(*reverseCenter)
    glTranslate(*center)

    glRotatef(angle[0], 1, 0, 0)
    glRotatef(angle[1], 0, 0, 1)
    glRotatef(-1 * angle[2], 0, 1, 0)
     
    # Position the camera at [0,0,20], looking at [0,0,0],
    # with [0,1,0] as the up direction.    

    color = [0.5, 0.5, 0.9, 1.0]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glutWireSphere(1, 30, 30)
    
    glPopMatrix()
    
    # --------------------------- POINTER ---------------------------

    if pointerPosition != None and currentlyPointing:
        glPushMatrix()
        glLoadIdentity()
        # Position the camera at [0,0,20], looking at [0,0,0],
        # with [0,1,0] as the up direction.
        gluLookAt(0,0,10,
                  0,0,0,
                  0,1,0)

        glTranslate(*center)        
        glTranslate(*pointerPosition)

        color = [1.0, 0.0, 0.0, 1.0]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glutSolidSphere(0.1, 30, 30)

        glPopMatrix()
    glutSwapBuffers()
    return


def initRendering():
    glEnable(GL_DEPTH_TEST);   # Depth testing must be turned on
    glEnable(GL_LIGHTING);     # Enable lighting calculations
    glEnable(GL_LIGHT0);       # Turn on light #0.
    
    glClearColor(0.,0.,0.,1.)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    lightZeroPosition = [10.,4.,10.,1.]
    lightZeroColor = [0.8,1.0,0.8,1.0] #green tinged
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor)
    glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.1)
    glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.05)
    glEnable(GL_LIGHT0)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(60.,1.,1.,60.)
    glMatrixMode(GL_MODELVIEW)
    
    # Position the camera at [0,0,10], looking at [0,0,0],
    # with [0,1,0] as the up direction.
    gluLookAt(0,0,10,
              0,0,0,
              0,1,0)
    glPushMatrix()

def openGLSetup():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800,800)
    glutInitWindowPosition(0,0)
    glutCreateWindow("Leap Visualizer")

    # Initialize OpenGL parameters.
    initRendering()
    
    # Set up callback functions for key presses
    glutKeyboardFunc(keyboardFunc) # Handles "normal" ascii symbols
    glutIdleFunc(updatePosition)
    
    #Call this whenever window needs redrawing
    glutDisplayFunc(drawScene)
    glutMainLoop()

def main():
    print "Press Enter to quit..."

    # Create a sample listener and controller
    listener = GestureController()
    controller = Leap.Controller()

    listener.setControlQueue(controlQueue)
    listener.setPointerQueue(pointerQueue)

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    openGLSetup()   
    
    return
    


if __name__ == "__main__":
    main()
