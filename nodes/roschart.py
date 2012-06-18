#!/usr/bin/env python

import multiprocessing
import os
import Queue
import signal

import roslib; roslib.load_manifest( 'roschart' )
import rospy

from roschart.msg import Point

from gi.repository import Gtk, GObject

class Listener (multiprocessing.Process):
    """Wait for rospy "roschart" messages."""
    def __init__( self ):
        multiprocessing.Process.__init__( self )
        self.queue = multiprocessing.Queue()

    def run( self ):
        rospy.init_node( 'roschart', anonymous=True )
        rospy.Subscriber( 'roschart', Point, self.data_callback )
        rospy.spin()

    def data_callback( self, point ):
        self.queue.put( point )


class Plotter (Gtk.Window):
    """Open a GTK window and respond to rospy input events."""
    def __init__( self ):
        Gtk.Window.__init__( self, title="roschart" )

        self.label = Gtk.Label( label="waiting for data..." )
        self.add( self.label )
        
        self.listener = Listener()
        self.listener.start()

        self.connect( "delete-event", self.quit )
        
        self.show_all()

    def start( self ):
        self.alive = True
        self.poll_queue()

    def poll_queue( self ):
        while self.alive:
            try:
                data = self.listener.queue.get( timeout=0.005 )
            except Queue.Empty:
                pass
            else:
                self.label.set_text( str( data ) )

            while Gtk.events_pending():
                Gtk.main_iteration()

    def quit( self, obj, data ):
        """Send an interrupt (Ctrl+C) to listener; wait for rospy shutdown."""
        self.alive = False
        
        os.kill( self.listener.pid, signal.SIGINT )
        self.listener.join()
# for Windows, where SIGINT is reputed not to exist:
# http://objectmix.com/python/387639-sending-cntrl-c.html
##To the OP: you can download the pywin32 package from sourceforge, and use
##win32api.GenerateConsoleCtrlEvent(win32con.CTRL_C_EVENT, pgid)
##or call the same function using ctypes.
##See http://msdn.microsoft.com/en-us/library/ms683155(VS.85).aspx for some
##important remarks.

        Gtk.main_quit()
        

if __name__ == '__main__':
    plotter = Plotter()
    plotter.start()

    # pressing Ctrl+C before closing the window hangs the main process,
    # but this appears to be the fault of this GTK setup, independent of ROS
