#!/usr/bin/env python3

import rospy

class BaseNode(Object):
    rate: rospy.Rate
    hertz: int = 10
    shutdown_requested: bool = False

    def wait_for_simulator(self):
        # Wait for the simulator to be ready. If simulator is not ready, then time will be stuck at zero
        while rospy.Time.now().to_sec() == 0:
            self.rate.sleep()

    def stop(self):
        pass

    def shutdown_hook(self):
        self.shutdown_requested = True
        print("\n**** Shutdown Requested ****")
        self.stop()

    def loop(self):
        pass

    def initial_setup(self):
        pass

    def run(self):
        self.rate = rospy.Rate(self.hertz)
        self.initial_setup()
        while not rospy.is_shutdown() and not self.shutdown_requested:
            self.loop()
            self.rate.sleep()
        self.stop()
        
if __name__ == '__main__':
    rn = BaseNode()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()

