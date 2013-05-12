import Leap, sys

firstHistoryFrame = 30
lastHistoryFrame = 40
numPastFrames = lastHistoryFrame - firstHistoryFrame

averageWindow = 5

fingerDecreaseThreshold = 2
fingerIncreaseThreshold = 1
radiusDecreaseThreshold = 5

palmVelocityThreshold = 700

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














def main():
    # Create a sample listener and controller
    listener = GestureController()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()

    # Remove the sample listener when done
    controller.remove_listener(listener)


if __name__ == "__main__":
    main()
