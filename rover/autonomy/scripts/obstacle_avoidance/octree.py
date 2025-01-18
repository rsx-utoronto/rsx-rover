class OctreeNode:
    def __init__(self, boundary, depth=0, max_depth=4):
        self.boundary = boundary  # A cube defined by min and max points
        self.depth = depth
        self.max_depth = max_depth
        self.points = []
        self.children = []

    def insert(self, point):
        if self.depth == self.max_depth:
            self.points.append(point)
            return
        
        if not self.children:
            self.subdivide()
        
        for child in self.children:
            if child.contains(point):
                child.insert(point)
                return
    
    def contains(self, point):
        x, y, z = point
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.boundary
        return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax
    
    def subdivide(self):
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.boundary
        xmid = (xmin + xmax) / 2
        ymid = (ymin + ymax) / 2
        zmid = (zmin + zmax) / 2

        self.children = [
            OctreeNode(((xmin, ymin, zmin), (xmid, ymid, zmid)), self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymin, zmin), (xmax, ymid, zmid)), self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymid, zmin), (xmid, ymax, zmid)), self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymid, zmin), (xmax, ymax, zmid)), self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymin, zmid), (xmid, ymid, zmax)), self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymin, zmid), (xmax, ymid, zmax)), self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymid, zmid), (xmid, ymax, zmax)), self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymid, zmid), (xmax, ymax, zmax)), self.depth + 1, self.max_depth),
        ]

# Example Usage
boundary = ((0, 0, 0), (10, 10, 10))
octree = OctreeNode(boundary)

points = [(1, 1, 1), (2, 2, 2), (8, 8, 8)]
for point in points:
    octree.insert(point)
