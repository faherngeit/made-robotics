from tkinter import *
import math
import heapq
# from sympy import Point, Polygon
from shapely.geometry import Point, Polygon

'''================= Your classes and methods ================='''

RESOLUTION = 40
ANGLE_LIMIT = math.pi / 4
ANGLE_TOLERENCE = math.pi / 12
CHNG_DIR_PENALTY = 1e3


def naive_heuristic(position, target):
    x, y, yaw = position
    xt, yt, yawt = target
    circ = 2 * math.pi
    dist = (x - xt) ** 2 + (y - yt) ** 2 + 10 * (yaw % circ - yawt % circ) ** 2
    return dist

def custom_heuristic(position, target):
    x, y, yaw = position
    xt, yt, yawt = target
    dist = (x - xt) ** 2 + (y - yt) ** 2
    angle = abs((yaw - yawt) % math.pi)
    return dist + angle

def create_circle(x, y, r, canvasName, color='yellow'): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvasName.create_oval(x0, y0, x1, y1, outline=color, fill=color)


def naive_path(position, target, canvas, radius=2, n_points=50):
    x, y, yaw = position
    xt, yt, yawt = target
    dx = (xt - x) / n_points
    dy = (yt - y) / n_points
    xc, yc = x, y
    for _ in range(n_points):
        xc += dx
        yc += dy
        create_circle(xc, yc, radius, canvas)


class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        # If item already in priority queue with higher priority, update its priority and rebuild the heap.
        # If item already in priority queue with equal or lower priority, do nothing.
        # If item not in priority queue, do the same thing as self.push.
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)


class Node:
    def __init__(self, position, target, direction=1, parent=None):
        self.pos = (int(position[0]), int(position[1]), position[2])
        self.target = target
        self.parent = parent
        self.direction = direction
        self.g = 0
        self.seen = False
        if parent is not None:
            self.g = parent.g + 1
        self.h = self.some_heurisitic()
        self.f = self.g + self.h
        self.children = []

    def append(self, child):
        self.children.append(child)

    def __eq__(self, other):
        if abs(self.pos[0] - other.pos[0]) < RESOLUTION // 4 and \
                abs(self.pos[1] - other.pos[1]) < RESOLUTION // 4 and\
                abs(self.pos[2] - other.pos[2]) < ANGLE_TOLERENCE / 2:
            return True
        else:
            return False

    def some_heurisitic(self):
        if self.parent is None:
            return custom_heuristic(self.pos, self.target)
        penalty = abs(self.direction - self.parent.direction)
        score = custom_heuristic(self.pos, self.target) + penalty * CHNG_DIR_PENALTY
        return score


# These functions will help you to check collisions with obstacles

def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new+cx, y_new+cy))

    return new_points

def get_polygon_from_position(position) :
    x,y,yaw = position
    # points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)]
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100), (x - 50, y - 100)]
    new_points = rotate(points, yaw * 180 / math.pi, (x,y))
    # return Polygon(*list(map(Point, new_points)))
    return Polygon(new_points)

def get_polygon_from_obstacle(obstacle) :
    # points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]), (obstacle[6], obstacle[7])]
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]),
              (obstacle[6], obstacle[7]), (obstacle[0], obstacle[1])]
    # return Polygon(*list(map(Point, points)))
    return Polygon(points)

def collides(position, obstacle) :
    return get_polygon_from_position(position).intersection(get_polygon_from_obstacle(obstacle))
        

class Window():

    def a_star_path(self):
        print('Path searching begins!')
        node_queue = PriorityQueue()
        trgt = self.get_target_position()
        root = Node(self.get_start_position(), trgt)
        node_queue.push(root, root.f)
        node_seen = []
        in_progress = True
        while in_progress:
            node = node_queue.pop()
            if node not in node_seen:
                node_seen.append(node)
                print(f"Node position: [{node.pos[0]}, {node.pos[1]}, {node.pos[2]}]")
                self.plot_node(node, color='cyan')
                self.canvas.update_idletasks()
                if self.is_target(node.pos):
                    return node
                self.move(node)
                for ch_node in node.children:
                    if ch_node not in node_seen:
                        node_queue.push(ch_node, ch_node.f)

    def move(self, node, n_chld=4):
        node.seen = True
        for stp in range(2 * n_chld + 1):
            for direction in range(2):
                x, y, yaw = node.pos
                yaw += (stp - n_chld) * ANGLE_LIMIT / n_chld
                x += (1 - 2 * direction) * math.cos(yaw) * RESOLUTION
                y += (1 - 2 * direction) * math.sin(yaw) * RESOLUTION
                pos = (x, y, yaw)
                if not self.valid_move(pos):
                    continue
                child = Node(pos, node.target, parent=node, direction=direction)
                if child not in node.children:
                    node.append(child)

    def valid_move(self, position):
        for obstacle in self.get_obstacles():
            if collides(position, obstacle):
                return False
        return True
    
    def is_target(self, pos):
        target = self.get_target_position()
        dx = abs(target[0] - pos[0])
        dy = abs(target[1] - pos[1])
        angle = abs(target[2] - pos[2])
        if dx < RESOLUTION and dy < RESOLUTION and angle < ANGLE_TOLERENCE:
            return True
        return False

    def plot_path(self, node):
        while node.parent is not None:
            x, y, _ = node.pos
            xe, ye, _ = node.parent.pos
            self.canvas.create_line(x, y, xe, ye, fill='yellow', width=3)
            node = node.parent

    def plot_node(self, node, color='yellow'):
        create_circle(node.pos[0], node.pos[1], 2, self.canvas, color=color)
        x = node.pos[0]
        y = node.pos[1]
        xe = x + math.cos(node.pos[2]) * RESOLUTION
        ye = y + math.sin(node.pos[2]) * RESOLUTION
        self.canvas.create_line(x, y, xe, ye, arrow=LAST)

    '''================= Your Main Function ================='''
    
    def go(self, event):
    
        # Write your code here
                
        print("Start position:", self.get_start_position())
        print("Target position:", self.get_target_position()) 
        print("Obstacles:", self.get_obstacles())
        
        # Example of collision calculation
        
        number_of_collisions = 0
        for obstacle in self.get_obstacles() :
            if collides(self.get_start_position(), obstacle) :
                number_of_collisions += 1
        print("Start position collides with", number_of_collisions, "obstacles")

        path = self.a_star_path()
        print("Optimal path is found!")
        self.plot_path(path)
        
    '''================= Interface Methods ================='''
    
    def get_obstacles(self) :
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2) :
                coords = self.canvas.coords(i)
                if coords and len(coords) > 4:
                    if abs(coords[0] - coords[2]) > RESOLUTION:
                        obstacles.append(coords)
        return obstacles
            
            
    def get_start_position(self) :
        x,y = self.get_center(2) # Purple block has id 2
        yaw = self.get_yaw(2)
        return x,y,yaw
    
    def get_target_position(self) :
        x,y = self.get_center(1) # Green block has id 1 
        yaw = self.get_yaw(1)
        return x,y,yaw

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2])/2
        end_y = (points[1] + points[3])/2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y 
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0 ) :
            return math.acos(cos_yaw) - math.pi / 2
        else:
            return -math.acos(cos_yaw) - math.pi / 2
       
    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new+cx)
            new_points.append(y_new+cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0: angle = math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))
        elif wx - x < 0: angle = -math.acos((cat1**2 + cat2**2 - hyp**2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0,0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width/2)
        self.create_purple_block(self.width/2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()
        
    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width  = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry(f'{self.width}x{self.height}')
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        # self.points = [0, 500, 500/2, 0, 500, 500]
    
if __name__ == "__main__":
    run = Window()
    run.run()
