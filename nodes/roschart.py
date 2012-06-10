#!/usr/bin/env python

import multiprocessing
import os
import signal

import roslib; roslib.load_manifest( 'roschart' )
import rospy

from roschart.msg import Point

from gi.repository import Gtk

class Listener (multiprocessing.Process):
    """Wait for rospy "roschart" messages."""
    def __init__( self, parent_callback ):
        multiprocessing.Process.__init__( self )
        self.queue = multiprocessing.Queue()
        self.parent_callback = parent_callback

    def run( self ):
        rospy.init_node( 'roschart', anonymous=True )
        rospy.Subscriber( 'roschart', Point, self.data_callback )
        rospy.spin()

    def data_callback( self, point ):
        self.queue.put( point )
        self.parent_callback()


class Plotter (Gtk.Window):
    """Open a GTK window and respond to rospy input events."""
    def __init__( self ):
        Gtk.Window.__init__( self, title="roschart" )
        self.label = Gtk.Label( label="waiting for data..." )
        self.add( self.label )
        
        self.listener = Listener( self.data_callback )
        self.listener.start()

        self.connect( "delete-event", Gtk.main_quit )
        self.show_all()

    def data_callback( self ):
        data = self.listener.queue.get()
        self.label.set_text( str( data ) )
        print str( data )

    def __del__( self ):
        """Send an interrupt (Ctrl+C) to listener; wait for rospy shutdown."""
        os.kill( self.listener.pid, signal.SIGINT )
        self.listener.join()
# for Windows, where SIGINT is reputed not to exist:
# http://objectmix.com/python/387639-sending-cntrl-c.html
##To the OP: you can download the pywin32 package from sourceforge, and use
##win32api.GenerateConsoleCtrlEvent(win32con.CTRL_C_EVENT, pgid)
##or call the same function using ctypes.
##See http://msdn.microsoft.com/en-us/library/ms683155(VS.85).aspx for some
##important remarks.
        

if __name__ == '__main__':
    win = Plotter()
    Gtk.main()
    #del win # this worked before the queue and callback function were added
    win.__del__() # now 'del win' doesn't call win.__del__()??

    # also, pressing Ctrl+C before closing the window hangs the main process
    # but this appears to be the fault of this GTK setup, independent of ROS
