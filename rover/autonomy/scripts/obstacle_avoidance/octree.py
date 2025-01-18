class OctreeNode:
    def __init__(self, boundary, resolution, depth=0, max_depth=4):
        """
        Octree Node Constructor.

        Args:
            boundary (tuple): ((xmin, ymin, zmin), (xmax, ymax, zmax)) defining the cube's boundaries.
            resolution (float): Minimum resolution or size of the smallest cube in the octree.
            depth (int): Current depth of this node in the tree.
            max_depth (int): Maximum depth allowed for the tree.
        """
        self.boundary = boundary  # The boundary of this node
        self.resolution = resolution  # Minimum size of subdivisions
        self.depth = depth  # Current depth of this node
        self.max_depth = max_depth  # Maximum depth of the tree
        self.points = []  # Points contained in this node
        self.children = []  # Subdivided children nodes

    def insert(self, point):
        """
        Insert a point into the octree.

        Args:
            point (tuple): A (x, y, z) coordinate.
        """
        # Check if the point fits within this node's boundary
        if not self.contains(point):
            return False

        # If the node is at the maximum depth or resolution, store the point
        if self.depth == self.max_depth or self._get_size() <= self.resolution:
            self.points.append(point)
            return True

        # Subdivide if not already subdivided
        if not self.children:
            self.subdivide()

        # Insert the point into the appropriate child node
        for child in self.children:
            if child.insert(point):
                return True
        return False

    def contains(self, point):
        """
        Check if a point lies within this node's boundary.

        Args:
            point (tuple): A (x, y, z) coordinate.

        Returns:
            bool: True if the point is inside the boundary, False otherwise.
        """
        x, y, z = point
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.boundary
        return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax

    def _get_size(self):
        """
        Calculate the size of this node (length of one side of the cube).

        Returns:
            float: Size of the cube.
        """
        (xmin, _, _), (xmax, _, _) = self.boundary
        return xmax - xmin

    def subdivide(self):
        """
        Subdivide the current node into eight children nodes.
        """
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.boundary
        xmid = (xmin + xmax) / 2
        ymid = (ymin + ymax) / 2
        zmid = (zmin + zmax) / 2

        self.children = [
            OctreeNode(((xmin, ymin, zmin), (xmid, ymid, zmid)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymin, zmin), (xmax, ymid, zmid)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymid, zmin), (xmid, ymax, zmid)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymid, zmin), (xmax, ymax, zmid)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymin, zmid), (xmid, ymid, zmax)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymin, zmid), (xmax, ymid, zmax)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmin, ymid, zmid), (xmid, ymax, zmax)), self.resolution, self.depth + 1, self.max_depth),
            OctreeNode(((xmid, ymid, zmid), (xmax, ymax, zmax)), self.resolution, self.depth + 1, self.max_depth),
        ]


# Example Usage
if __name__ == "__main__":
    # Define the boundary for the root node
    boundary = ((0, 0, 0), (10, 10, 10))

    # Define the resolution of the octree
    tree_resolution = 0.5  # Minimum size of the smallest cube

    # Create the octree with resolution
    octree = OctreeNode(boundary, tree_resolution)

    # Insert some points
    points = [(1, 1, 1), (2, 2, 2), (8, 8, 8)]
    for point in points:
        octree.insert(point)
