
class Point:
    """An imutable point class used to represent a 2 dimensional point in space.
    """
    
    def __init__(self, x, y):
        self.__x = x
        self.__y = y
        
    def __add__(self, *args):
        """Create a new point by adding to this one.
        
        Args:
            args: May be an instance of Point or an array_like object with two number objects.
            
        Returns:
            A new point object.
        """
        
        if type(args[0]) is type(Point):
            return type(self)(self.x + args[0].x, self.y + args[0].y)
        else:
            return type(self)(self.x + args[0][0], self.y + args[0][1])
        
    def __subtract(self, point):
        """Create a new point by subtracting from this one.
        
        Args:
            args: May be an instance of Point or an array_like object with two number objects.
            
        Returns:
            A new point object.
        """
        
        if type(args[0]) is type(Point):
            return type(self)(self.x - args[0].x, self.y - args[0].y)
        else:
            return type(self)(self.x - args[0][0], self.y - args[0][1])
    
    def __copy__(self):
        return type(self)(self.x, self.y)
    
    def __deepcopy__(self, memo):
        return type(self)(self.x, self.y)
    
    @property
    def x(self):
        return self.__x
    
    @property
    def y(self):
        return self.__y
        
class Size(Point):
    """An imutable size class used to represent the size of a 2 dimensional object.
    """

    def __init__(self, w, h):
        Point.__init__(self, w, h)
        
    @property
    def width(self):
        return self.x
        
    @property
    def height(self):
        return self.y
        
class Rect:
    """An immutable rectangle class used to represent a rectangle in 2 dimensional space.
    """
    
    def __init__(self, origin : Point, size : Size):
        self.origin = origin
        self.size = size
        
    def shift(self, amount : Point):
        """Creates a new Rect by shifting this one by the specified amount.
        
        Args:
            amount: A point representing the amount to shift the origin of this rect by.
            
        Returns:
            A new rect object.
        """
        
        return type(self)(self.origin + amount)
        
    def grow(self, amount : Size):
        """Creates a new Rect by growing this one by the specified amount.
        
        Args:
            amount: A point representing the amount to grow the size of this rect by.
            
        Returns:
            A new rect object.
        """
        
        return type(self)(self.size + amount)
        
    def __copy__(self):
        return type(self)(self.origin, self.size)
    
    def __deepcopy__(self, memo):
        return type(self)(self.origin, self.size)