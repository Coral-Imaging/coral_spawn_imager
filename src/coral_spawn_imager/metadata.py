#! /usr/bin/env python3

import json
import numpy as np
# import tifffile
import PIL.Image as PIL_Image
from PIL.PngImagePlugin import PngInfo
from pprint import *

from coral_spawn_imager.PiCamera2Wrapper import PiCamera2Wrapper

# create custom description for metadata
# info = {"species": "acropora", 
#         "tank": "TANKID", 
#         "batch": "BATCHID",
#         "geneid": "GENEID"}



CORAL_METADATA_FILE = '/home/cslics04/cslics_ws/src/coral_spawn_imager/launch/coral_metadata.json'

# code to test out tiff file metadata appending to the image
picam = PiCamera2Wrapper()

coral_metadata = picam.read_custom_metadata(CORAL_METADATA_FILE)
print(coral_metadata)
img, img_name, pi_metadata = picam.capture_image(format='png')

picam.update_metadata(pi_metadata, coral_metadata)

# metadata.update(info)
picam.save_image(img, img_name, pi_metadata)

# metadata = PngInfo()
# metadata.add_text("NewStr", "A string")
# metadata.add_text("NewInt", str(1234))

# img_pil = PIL_Image.fromarray(img)
# img_name_pil = img_name[:-5] + '_pil.png'
# img_pil.save(img_name_pil, pnginfo=metadata)


# info_str = json.dumps(info)
# print(info_str)
# img_name_new = img_name[:-5] + '_md.png'

# write to file
# tifffile.imsave(img_name_new, img, description=info_str)
img_md = PIL_Image.open(img_name)
pprint(img_md.text)

# print(exif_data)
# read the file
# img_md = tifffile.TiffFile(img_name_new)
# page = img_md.pages[0]

# print file description:
# print(page.tags["ImageDescription"].value)


import code
code.interact(local=dict(globals(), **locals()))
