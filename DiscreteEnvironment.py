import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = float(resolution)

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        if nid < 0 or nid > numpy.prod(self.num_cells)-1 :
            return None
        
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        config = numpy.array(config).astype(float)
        
        offset = numpy.subtract(config,self.lower_limits)
        coord = numpy.divide(offset,self.resolution)

        coord = numpy.minimum(coord, numpy.subtract(self.num_cells,1))
        coord = coord.astype(int)
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        coord = numpy.array(coord).astype(float)
        config = self.lower_limits + numpy.multiply(coord,self.resolution)
        
        for dim in range(0, len(coord)):
            if coord[dim] == self.num_cells[dim]-1:
                config[dim] += (self.upper_limits[dim]-config[dim])/2
            else:
                config[dim] += self.resolution/2

        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = numpy.ravel_multi_index(coord,self.num_cells,order='F')
        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        if node_id < 0 or node_id > numpy.prod(self.num_cells)-1 :
            return None
        
        return numpy.array(numpy.unravel_index(node_id,self.num_cells,order='F'))
        
        
        
