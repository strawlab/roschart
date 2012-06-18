#!/usr/bin/env python

import threading

import roslib; roslib.load_manifest( 'roschart' )
import rospy

from roschart.msg import Point

from gi.repository import Gtk, Gdk, GObject, GLib

GObject.threads_init()
Gdk.threads_init()

class Plotter (Gtk.Window):
    UPDATE_FREQ = 30
    """Open a GTK window and respond to rospy input events."""
    def __init__( self ):
        Gtk.Window.__init__( self, title="roschart" )

        self.label = Gtk.Label( label="waiting for data..." )
        self.add( self.label )
        self.connect( "delete-event", Gtk.main_quit )

        self.point = Point()
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber( 'roschart', Point, self.data_callback )
        
        self.show_all()

    def data_callback(self, msg):
        with self.lock:
            self.point = msg

    def update_ui(self):
        with self.lock:
            self.label.set_text(str(self.point))
        return True

    def start( self ):
        rosthread = threading.Thread(name="ros spin thread", target=rospy.spin)
        rosthread.daemon = True
        rosthread.start()
        GLib.timeout_add(1000/self.UPDATE_FREQ, self.update_ui)
        Gtk.main()


if __name__ == '__main__':
    rospy.init_node( 'roschart', anonymous=True, disable_signals=True )
    plotter = Plotter()
    plotter.start()
    rospy.signal_shutdown('quit from gui')

