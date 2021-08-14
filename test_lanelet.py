# TODO example not to be commited in this location

import lanelet2
import os


if __name__ == "__main__":
  path = os.path.abspath(lanelet2.__file__)
  print("Lanelet2 Path: " + str(path))
  proj = lanelet2.projection.LocalFrameProjector("+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs", lanelet2.io.Origin(0,0))
  laneletmap = lanelet2.io.load("/workspaces/carma_ws/autoware.ai/AOI_1_TFHRC_pretty.osm", proj)

  speed_limit_reg = laneletmap.regulatoryElementLayer.get(12800) # Get a DigitalSpeedLimit
  print("SpeedLimit: " + str(speed_limit_reg.getSpeedLimit()))
