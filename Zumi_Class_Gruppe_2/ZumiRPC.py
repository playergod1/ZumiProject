"""
Zumi RPC class with advanced methods
like Random Walk, move to Point (x, y)

Example usage can be found in notebook example ``unet.ipynb``.
"""

import os
import rpyc
import numpy as np
import pandas as pd
import random
import cv2
from datetime import datetime
from GPSPhoto import gpsphoto
from Position import Position


class Zumi(object):
    def __init__(self, direction: float, port: int, ip: str ='keuper-labs.org'):
        self.measurements = None
        self.IP = ip
        self.PORT = port
        zumi_no = int(int(self.PORT) - 9000)

        # Connecting via RPC
        rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True
        conn = rpyc.connect(ip, port)
        self.zumi = conn.root

        self.position = Position(zumi_no, direction)

        # Creating directory for zumi data
        self.directory = "Zumi_{}".format(port - 9000)
        try:
            os.makedirs(self.directory)
        except FileExistsError:
            # directory already exists
            pass

        # Creating directory for top-cam data
        self.directory_two = "{}/Data_from_{}".format(self.directory, datetime.now().strftime("%m-%d-%Y"))
        try:
            os.makedirs(self.directory_two)
        except FileExistsError:
            # directory already exists
            pass
        self.csv_name = "{}/IR_from_{}.csv".format(self.directory_two, datetime.now().strftime("%H-%M-%S"))
        # self.csv_name = "{}/IR_from_{}.csv".format(self.directory,datetime.now().strftime("%H-%M-%S"))
        print("Zumi Initialized")

    def ping(self):
        """ Ping zumi car """
        return self.zumi.ping()

    def get_all_IR_data(self):
        """ Reads and saves IR_data """
        col_names = ['year', 'month', 'day', 'hour', 'minute', 'second',
                     'front right', 'bottom right', 'back right',
                     'bottom left', 'back left', 'front left']
        now = datetime.now()
        time = [now.date().year, now.date().month, now.date().day, now.time().hour, now.time().minute,
                now.time().second]
        data = list(self.zumi.get_all_IR_data())
        time.extend(data)
        new_data = pd.DataFrame([time], columns=col_names)
        for column in new_data.columns:
            new_data[column] = new_data[column].astype('int16')
        if self.measurements is None:
            self.measurements = new_data
            self.measurements.to_csv(self.csv_name, index=False)
        else:
            self.measurements = self.measurements.append(new_data)
            new_data.to_csv(self.csv_name, index=False, mode='a')
        return data

    def get_picture(self):
        """
        Takes and saves a picture from zumi front cam
        GPS Information are added to the image
        """
        frame = self.zumi.get_picture()

        jpg_name = "{}/Zumicam_{}.jpeg".format(self.directory_two, datetime.now().strftime("%H-%M-%S"))
        jpg_name_gps = "{}/Zumicam_{}_GPS.jpeg".format(self.directory_two, datetime.now().strftime("%H-%M-%S"))

        cv2.imwrite(jpg_name, img=np.array(frame))

        photo = gpsphoto.GPSPhoto(jpg_name)
        info = gpsphoto.GPSInfo(((float(self.position.last_y) / 100), (float(self.position.last_x) / 100)))
        photo.modGPSData(info, jpg_name_gps)
        return frame

    def get_pos_and_dir(self):
        """ Returns x,y pos (as list) and direction"""
        x = self.position.last_x
        y = self.position.last_y
        pos = [x, y]
        return pos, self.position.direction

    def calc_pos_and_dir(self):
        """ Calculates and returns the new position and direction of the zumi """
        pos, direction = self.position.calc_current_position()
        return pos, direction

    def get_Sensors(self, recalculate_direction: bool = False):
        """ Used to update position and direction of zumi"""
        data = self.get_all_IR_data()
        """ Since the direction gets calculated out of the pixel difference between new and old position, 
            even a one pixel difference in a new position calculation can lead to a wrong direction calculation
            Direction recalculation is only used with forward to compensate drift."""
        if recalculate_direction:
            pos, direction = self.calc_pos_and_dir()
        else:
            pos, direction = self.get_pos_and_dir()
        ###
        frame = None #can be replaced with self.get_picture(), but some zumi cameras are currently not working, therefore it's currently not used
        ###
        return frame, data, pos, direction

    def forward(
            self,
            speed: float = 40,
            duration: float = 1.0,
            correction: float = 4,
            repeat: int = 1,
            check_clearance: bool = False,
    ) -> bool:
        """ Drive zumi forward with given correction

        Args:
            speed: Zumi speed
            duration: Duration of maneuver
            correction: Correction angle to drive straight
            repeat: How often forward should be executed
            check_clearance: Enables, check something is in front
        """
        path_clear = True
        """predict endposition, in case that zumi position can't be found afterwards via overhead camera"""
        dist = self.get_distance_for_duration(duration * repeat, speed)
        end_point = self.position.predict_point_from_current_position(dist)
        self.position.set_prediction(end_point[0], end_point[1])
        if check_clearance:
            """check if path and endpoint stays on the street"""
            path_clear = self.position.check_path_in_front(dist)
        if path_clear:
            for i in range(0, repeat):
                if correction != 0:
                    """prevent drift by correcting direction before forward"""
                    self.turn(angle=correction, update_the_direction=False)
                self.zumi.forward(speed=speed, duration=duration)
            print("Zumi Forward")
            self.get_Sensors(recalculate_direction=True)
        else:
            print("Obstacle in Path!")
        return path_clear

    def reverse(self, speed: float = 20, duration: float = 1.0) -> None:
        """ Driving reverse and updating sensors """
        self.zumi.reverse(speed=speed, duration=duration)
        print("Zumi Reverse")
        self.get_Sensors(recalculate_direction=True)
        return

    def turn(
            self,
            angle: float,
            repeat: int = 1,
            duration: float = 1.0,
            update_the_direction: bool = True,
    ) -> None:
        """ Turning either left (negative angle) or right (positive angle) """
        if angle < 0:
            self.turn_left((-angle), repeat=repeat, duration=duration, update_the_direction=update_the_direction)
        else:
            self.turn_right(angle, repeat=repeat, duration=duration, update_the_direction=update_the_direction)
        return

    def turn_left(
            self,
            angle: float,
            repeat: int = 1,
            duration: float = 1.0,
            update_the_direction: bool = True,
    ) -> None:
        """ Turning left with angle and updating position and direction """
        for i in range(0, repeat):
            if angle > 90:
                duration = angle / 90
            for i in range(0, repeat):
                self.zumi.turn_left(angle, duration=duration)

            if update_the_direction:
                self.position.update_direction((-angle))
                print("Zumi Turn Left")
            self.get_Sensors(recalculate_direction=False)
        return

    def turn_right(
            self,
            angle: float,
            repeat: int = 1,
            duration: float = 1.0,
            update_the_direction: bool = True,
    ) -> None:
        """ Turning right with angle and updating position and direction """
        for i in range(0, repeat):
            if angle > 90:
                duration = angle / 90
            for i in range(0, repeat):
                self.zumi.turn_right(angle, duration=duration)

            if update_the_direction:
                self.position.update_direction(angle)
                print("Zumi Turn Right")
            self.get_Sensors(recalculate_direction=False)
        return

    def get_duration_for_distance(self, distance: float, speed: float = 40):
        """
        How long zumi needs to drive
        TODO: Add formula for other speed than 40
        """
        if speed == 40:
            duration = round(((distance + 20) / 130), 1)
        else:
            duration = round(((distance + 20) / 130), 1)
            print(" Speed != 40 not implemented. Using formula for speed 40 ")
        return round(duration, 1)

    def get_distance_for_duration(self, duration: float, speed: float = 40):
        """
        How far zumi drives with given duration
        TODO: Add formula for other speed than 40
        """
        if speed == 40:
            distance = 130 * duration + 20
        else:
            distance = 130 * duration + 20
            print(" Speed != 40 not implemented. Using formula for speed 40 ")
        return int(distance)

    def drive_towards(self, x: int, y: int):
        """
        Drive zumi to point on a direct line. 
        Takes care of turning zumi towards point and the duration calculation for forward
        TODO: (if necessary) drive from last point in route to x, y
        """
        self.turn_towards(x, y)
        distance = self.position.get_distance_to(x, y)
        duration = self.get_duration_for_distance(distance)
        print("Distance towards {},{} is {}".format(x, y, distance))
        print("Duration for drive towards {},{} is {}".format(x, y, duration))
        self.forward(duration=duration, correction=3)
        distance = self.position.get_distance_to(x, y)
        print("After drive Distance to {},{} is {}".format(x, y, distance))
        return distance

    def drive_random(self, iterations: int = 30, duration: float = 1.0) -> None:
        """
        Drive zumi random on street:
        Checks if something is in front, if not it drive a small step,
        if something is in front it calculates a random correct angle
        todo: Sometime stuck, because global position is not always so exact
        """
        for i in range(iterations):
            # Drive forward if nothing is in front
            possible = self.forward(duration=duration, check_clearance=True)
            print("Direction = ", self.position.direction)

            # Calculate new angle until find a good one, then drive forward
            while not possible:
                angle = random.randint(-180, 180)
                future_direction = self.position.get_valid_angle((self.position.direction + angle))
                print("Direction while not possible = ", self.position.direction)

                dist = self.get_distance_for_duration(duration, 40)
                valid = self.position.check_path_in_front(dist, future_direction)
                if valid:
                    self.turn(angle)
                    possible = self.forward(duration=duration, check_clearance=True)
        return

    def get_distance_to(self, x: int, y: int):
        """ returns pixel distance to point (x, y)"""
        return self.position.get_distance_to(x, y)

    def get_battery_percentage(self):
        """ prints and returns battery percentage"""
        battery_level = self.zumi.get_battery_percentage()
        print("Battery Percentage = {}".format(battery_level))
        return battery_level

    def get_battery_voltage(self):
        """ prints and returns battery voltage """
        battery_level = self.zumi.get_battery_voltage()
        print("Battery Voltage = {}".format(battery_level))
        return battery_level

    def turn_towards(self, x: int, y: int):
        """calculate direction towards point, calculate difference to current direction and turns towards that point"""
        angle = self.position.calc_turnangle_towards(x, y)
        self.turn(angle)

    # mirroring some zumi functions
    def hard_brake(self):
        self.zumi.hard_brake()
        return

    def right_circle(self, speed: float = 30, step: int = 2):
        self.zumi.right_circle(speed, step)
        self.get_Sensors()
        return

    def left_circle(self, speed: float = 30, step: int = 2):
        self.zumi.left_circle(speed, step)
        self.get_Sensors()
        return

    def right_u_turn(self, speed: float = 30, step: int = 4, delay: float = 0.02):
        self.zumi.right_u_turn(speed, step, delay)
        self.get_Sensors()
        return

    def left_u_turn(self, speed: float = 30, step: int = 4, delay: float = 0.02):
        self.zumi.left_u_turn(speed, step, delay)
        self.get_Sensors()
        return

    
