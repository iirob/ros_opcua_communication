#!/usr/bin/env python

# Thanks to:
#  https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py

import sys
import time

from opcua import ua, Client


def data_change(handle, node, val, attr):
    print("Python: New data change event", handle, node, val, attr)


def event(handle, eventhandled):
    print("Python: New event", handle, eventhandled)


class SubHandler(object):
    pass


def main(args):
    client = Client("opc.tcp://localhost:21554")
    client.connect()
    root = client.get_root_node()
    # getting a variable by path and setting its value attribute
    myvar = client.get_node(ua.NodeId(2001, 2))
    datavalue = ua.Variant('Bok', ua.VariantType.String)
    # datavalue.SourceTimestamp = 0
    # datavalue.ServerTimestamp = 0
    myvar.set_value(datavalue)

    # mode = client.get_node("ns=2;s=3001.Mode")
    # mode.set_value(32, ua.VariantType.Int32)

    # subscribing to data change event to our variable
    handler = SubHandler()
    sub = client.create_subscription(500, handler)
    sub.subscribe_data_change(myvar)

    time.sleep(10)

    client.disconnect()


if __name__ == "__main__":
    main(sys.argv)
