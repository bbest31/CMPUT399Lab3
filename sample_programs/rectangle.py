class Rectangle:
    #Rectangle array is assumed to be in OpenCV standard representation of a rectange
    #   that is, an array of 4 elements: [x, y, width, height] where each quantity is given in pixels 
    #   and x,y correspond to the coordinate of the top left corner of the rectangle.
    def __init__(self,rectangle_array):
        self.array_representation = rectangle_array
        self.width = rectangle_array[2]
        self.height = rectangle_array[3]
        self.top_left = (rectangle_array[0], rectangle_array[1])
        self.top_right = (rectangle_array[0] + self.width , rectangle_array[1])
        self.bottom_left = (rectangle_array[0] , rectangle_array[1] + self.height)
        self.bottom_right = (rectangle_array[0] + self.width , rectangle_array[1] + self.height)
        self.centre = (top_left[0] + width/2, top_left[1] + height/2)
