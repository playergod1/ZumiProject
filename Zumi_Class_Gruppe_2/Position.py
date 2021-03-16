"""
Position helper class with x, y, and direction
"""

import os
import numpy as np
import math
from datetime import datetime
import matplotlib.pyplot as plt
import cv2


class Position(object):
    def __init__(self, number: int, direction: float):
        self.number = number
        self.last_x = None
        self.last_y = None
        self.predicted_x = None
        self.predicted_y = None
        self.direction = direction
        self.directory = "Overhead_{}".format(datetime.now().strftime("%m-%d-%Y"))
        try:
            os.makedirs(self.directory)
        except FileExistsError:
            # directory already exists
            pass

        self.calc_current_position()

    def set_direction(self, deg: float) -> None:
        """ sets a new valid direction """
        self.direction = self.get_valid_angle(deg)
        return

    def set_prediction(self, x: int, y: int) -> None:
        """ Set new estimated position """
        self.predicted_x = x
        self.predicted_y = y
        return

    def get_valid_angle(self, angle: float) -> float:
        """ Takes angle and formats it to range [-180, 180] """
        if angle <= -180:
            angle = 360 + angle
        elif angle > 180:
            angle = -360 + angle
        return angle

    def update_xy(self, pos: list) -> None:
        """ Updates actual position """
        self.last_x = pos[0]
        self.last_y = pos[1]
        return

    def update_direction(self, diff_deg: float) -> None:
        """ Updates direction with given angle """
        new_degree = self.direction + diff_deg
        new_degree = self.get_valid_angle(new_degree)
        self.direction = self.get_valid_angle(new_degree)
        return

    def get_distance_between_points(self, a_x: int, a_y: int, b_x: int, b_y: int) -> float:
        """ Calculates the euclidean distance between two points """
        delta_x = abs(b_x - a_x)
        delta_y = abs(b_y - a_y)
        return math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))

    def get_distance_to(self, x: int, y: int) -> float:
        """ Calculates distance to point (x, y) from current position"""
        return self.get_distance_between_points(x, y, self.last_x, self.last_y)

    def point_in_street(self, point, mask_path="street_mask.png") -> bool:
        """ Checks if point is in mask """
        mask = cv2.imread(mask_path, 0)
        return mask[point[1], point[0]] != 0

    def direction_between_points(self, new_x: int, new_y: int, old_x: int, old_y: int) -> int:
        """ Return the to_have direction between two points """
        delta_x = new_x - old_x
        delta_y = new_y - old_y
        return int(math.degrees(math.atan2(delta_y, delta_x)))

    def direction_from_last(self, new_x: int, new_y: int):
        """ Returns the direction between last point and new point """
        return self.direction_between_points(new_x, new_y, self.last_x, self.last_y)

    def calc_turnangle(self, deg: float, new_deg: float):
        """ Calculates the turnangle between to directions"""
        if deg < new_deg:
            right = new_deg - deg
            left = (360 - right)
        else:
            left = deg - new_deg
            right = (360 - left)
        if right > left:
            return -1 * left
        else:
            return right

    def get_overhead(self):
        """ Returns the overhead image """
        cap = cv2.VideoCapture('https://student:zumi2020@keuper-labs.org/zumi-cam')
        ret, image = cap.read()  # return a single frame in variable `frame`
        jpg_name = "{}/Overhead_{}.jpeg".format(self.directory, datetime.now().strftime("%H-%M-%S"))
        cv2.imwrite(jpg_name, img=image)
        return image

    def predict_point(self, start_point, angle, length):
        """ Calculates result point from point with lenght and angle """
        return (
            int(round(start_point[0] + length * math.cos(angle * math.pi / 180.0))),
            int(round(start_point[1] + length * math.sin(angle * math.pi / 180.0)))
        )

    def predict_point_from_current_position(self, length, angle=None):
        start_point = [self.last_x, self.last_y]
        if angle is None:
            return self.predict_point(start_point, self.direction, length)
        else:
            return self.predict_point(start_point, angle, length)

    def check_path_in_front(self, distance, angle=None):
        """ Checks if zumi can drive forward """
        current_angle = None
        if angle is None:
            current_angle = self.direction
        else:
            current_angle = angle
        # print(current_angle)
        """calculate angle left and right, to get a point left and right of the center of zumi given its current direction"""
        angle_to_left = self.get_valid_angle(current_angle - 90)
        angle_to_right = self.get_valid_angle(current_angle + 90)
        start_point = [self.last_x, self.last_y]
        left_point = self.predict_point(start_point, angle_to_left, 10)
        right_point = self.predict_point(start_point, angle_to_right, 10)
        start_list = [left_point, start_point, right_point]
        pixel_distances = list(range(10, distance, 5))
        clear = True
        """Check if all points in its pathway are within street, while trying to take the width of zumi into account"""
        for distance in pixel_distances:
            for point in start_list:
                if not self.point_in_street(self.predict_point(point, current_angle, distance)):
                    return False
        return clear

    def calc_current_direction(self, pos):
        if not self.last_x is None:
            deg = self.direction_from_last(pos[0], pos[1])
            self.set_direction(deg)
        self.update_xy(pos)
        return self.direction

    def calc_current_position(self, show=False, mask_path="detect_zumi_mask.png"):
        """ Calculates the current position from the Topcam """
        top_image = self.get_overhead()
        """ Apply street mask"""
        mask = cv2.imread(mask_path, 0)
        masked_image = cv2.bitwise_and(top_image, top_image, mask=mask)
        """Change colour space and filter colour"""
        hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
        lower_range, upper_range = self.get_lower_upper_colorvalue()
        mask = cv2.inRange(hsv, lower_range, upper_range)
        """ get contours, check if they have a reasonable size and get there center point"""
        (contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        points = list()
        for contour in contours:
            area = cv2.contourArea(contour)
            # print(area)
            if area > 350 and area < 1000:
                # print("asd")
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(mask, (x, y), (x + w, y + h), (255, 255, 255), 2)
                # print(x, y, w, h)
                points.append((x + 0.5 * w, y + 0.5 * h))
        global_pos = np.zeros(2)
        if len(points) > 0:
            for point in points:
                global_pos[0] += point[0]
                global_pos[1] += point[1]
            global_pos = [int(coord / len(points)) for coord in global_pos]
        else:
            print("Zumi not found, using predicted position")
            #might be the case if zumi is hidden behind a house, yet still on the street
            if self.predicted_x is not None:
                global_pos[0] = self.predicted_x
                global_pos[1] = self.predicted_y
            else:
                print("Zumi not found and no predicted position available")
        if show:
            #nice to have but no necessity, takes quite some time to calculate
            image = cv2.circle(top_image, (global_pos[0], global_pos[1]), radius=15, color=(255, 0, 0), thickness=-1)
            plt.figure(figsize=(15, 15))
            plt.imshow(top_image)
        direction = self.calc_current_direction(global_pos)
        return global_pos, direction

    def get_lower_upper_colorvalue(self):
        """ Maps zumi number to color range """
        if (self.number == 2) or (self.number == 5):
            lower_range = np.array([107, 135, 25])
            upper_range = np.array([121, 255, 255])
        elif (self.number == 1) or (self.number == 3):
            lower_range = np.array([0, 88, 70])
            upper_range = np.array([7, 229, 179])
        elif (self.number == 4) or (self.number == 6):
            lower_range = np.array([16, 102, 107])
            upper_range = np.array([39, 218, 199])
        return lower_range, upper_range

    def calc_turnangle_towards(self, x, y):
        """ calculate turnangle from current position and direction towards point"""
        new_deg = self.direction_from_last(x, y)
        print("Direction towards {},{} is {}".format(x, y, new_deg))
        turnangle = self.calc_turnangle(self.direction, new_deg)
        print("Turnangle towards {},{} is {}".format(x, y, turnangle))
        return turnangle
