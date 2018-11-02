class Rectangle:
    #Rectangle array is assumed to be in OpenCV standard representation of a rectange
    #   that is, an array of 4 elements: [x, y, width, height] where each quantity is given in pixels 
    #   and x,y correspond to the coordinate of the top left corner of the rectangle.
    def __init__(self,rectangle_array):
        self.array_representation = rectangle_array
        self.width = int(rectangle_array[2])
        self.height = int(rectangle_array[3])
        self.top_left = (int(rectangle_array[0]), int(rectangle_array[1]))
        self.top_right = (int(rectangle_array[0]) + self.width , int(rectangle_array[1]))
        self.bottom_left = (int(rectangle_array[0]) , int(rectangle_array[1]) + self.height)
        self.bottom_right = (int(rectangle_array[0]) + self.width , int(rectangle_array[1]) + self.height)
        self.centre = (self.top_left[0] + int(self.width/2), self.top_left[1] + int(self.height/2))
