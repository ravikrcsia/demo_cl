#!/usr/bin/env python3

import usbrelay_py
import time
from usb_relay.srv import RelaySw
import rclpy
from rclpy.node import Node

class UsbRelayService(Node):

    def __init__(self):
        super().__init__('usb_relay_service')
        self.srv = self.create_service(RelaySw, 'usb_relay_sw', self.usb_relay_sw_callback)

    def usb_relay_sw_callback(self, request, response):
        count = usbrelay_py.board_count()
        boards = usbrelay_py.board_details()
        if request.relaychannel == True:
            print('Incoming message for relay: 1',"State: ",request.relaystate)
            relay_channel_number = 2
        else:
            print('Incoming message for relay: 0',"State: ",request.relaystate)
            relay_channel_number = 1

        if not boards:
            result="Unable to connect to board of the relay"
            response.success = False
            response.message = result
        else:
            result = usbrelay_py.board_control(boards[0][0],relay_channel_number,request.relaystate)        
            response.success = True
            response.message = str(result)
            #self.get_logger().info('Incoming request',request.data)

        return response


def main(args=None):
    rclpy.init(args=args)

    service = UsbRelayService()

    rclpy.spin(service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()