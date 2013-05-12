from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import Leap, sys

firstHistoryFrame = 30
lastHistoryFrame = 40
numPastFrames = lastHistoryFrame - firstHistoryFrame

averageWindow = 5

fingerDecreaseThreshold = 2
fingerIncreaseThreshold = 1
radiusDecreaseThreshold = 5

palmVelocityThreshold = 700

center = [0,0,0]

class GestureController(Leap.Listener):
    def on_init(self, controller):
        self.state = 'open'
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def average_hands(self, controller, start, stop):
        hand = {
            'sphere_radius': 0,
            'numFingers': 0
        }
        for i in xrange(start, stop, 1):
            frame = controller.frame(i)
            if len(frame.hands) == 0:
                return None
            thisHand = frame.hands[0]
            hand['sphere_radius'] += thisHand.sphere_radius
            hand['numFingers'] += len(thisHand.fingers)

        hand = {
            'sphere_radius': hand['sphere_radius'] / (stop - start),
            'numFingers': float(hand['numFingers']) / (stop - start)
        }
        return hand

    def recent_hands(self, controller, count):
        for i in xrange(0, count, 1):
            frame = controller.frame(i)
            if len(frame.hands) != 0:
                return True
        return False

    def on_frame(self, controller):
        frame = controller.frame()

        # print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #       frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

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
                            self.state = 'closed'
                            return

                    # print frame.hands[0]

                else:
                    hand = self.average_hands(controller, 0, averageWindow)
                    oldHand = self.average_hands(controller, firstHistoryFrame, lastHistoryFrame)

                    if hand != None and oldHand != None:
                        if oldHand['numFingers'] - hand['numFingers'] < -1 * fingerIncreaseThreshold \
                                and oldHand['sphere_radius'] - hand['sphere_radius'] < -1 * radiusDecreaseThreshold:
                            print "Opening Gesture Detected! \n\n"
                            self.state = 'open'
                            return

                    hand = frame.hands[0]
                    print hand.palm_position
                    print (hand.direction.pitch * Leap.RAD_TO_DEG,
                            hand.palm_normal.roll * Leap.RAD_TO_DEG,
                            hand.direction.yaw * Leap.RAD_TO_DEG)

        else:
            if not self.recent_hands(controller, 10):
                self.state = 'open'





 
# This function is called whenever a "Normal" key press is received.
def keyboardFunc( key, x, y ):
    # exit if and ENTER or ESC is pressed
    if key == '\r' or key == '\x1b':
        exit()
    elif key == 'r':
        glTranslatef(-center[0], -center[1], -center[2])
        glRotatef(30.0, 1, 0, 0)
        glTranslatef(center[0], center[1], center[2])
        glutPostRedisplay()
    elif key == 't':
        glTranslatef(1,0,0)
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
    glPushMatrix()
    
    # Position the camera at [0,0,20], looking at [0,0,0],
    # with [0,1,0] as the up direction.
    
    color = [0.5, 0.5, 0.9, 1.0]
    glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
    glutWireTeapot(1)
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
    gluPerspective(40.,1.,1.,40.)
    glMatrixMode(GL_MODELVIEW)
    gluLookAt(0,0,20,
              0,0,0,
              0,1,0)
    glPushMatrix()


def main():
    print "Press Enter to quit..."
  
    # Create a sample listener and controller
    listener = GestureController()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800,800)
    glutInitWindowPosition(0,0)
    glutCreateWindow("Leap Visualizer")

    # Initialize OpenGL parameters.
    initRendering()
    
    # Set up callback functions for key presses
    glutKeyboardFunc(keyboardFunc) # Handles "normal" ascii symbols
    
    #Call this whenever window needs redrawing
    glutDisplayFunc(drawScene)
    
    glutMainLoop()
    return
    


if __name__ == "__main__":
    main()
