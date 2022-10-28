#! /usr/bin/env python3

import json
import numpy as np
import tifffile
from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper

# code to test out tiff file metadata appending to the image

picam = PiCamera2Wrapper()
img, img_name, metadata = picam.capture_image(format='tiff')

# create custom description
info = {"coral species": "acropora", 
        "tank": "TANKID", 
        "batch": "BATCHID",
        "geneid": "GENEID"}
info_str = json.dumps(info)
print(info_str)
img_name_new = img_name[:-5] + '_md.tiff'

# write to tiff file
tifffile.imsave(img_name_new, img, description=info_str)

# read the file
img_md = tifffile.TiffFile(img_name_new)
page = img_md.pages[0]

# print file description:
print(page.tags["ImageDescription"].value)


import code
code.interact(local=dict(globals(), **locals()))
