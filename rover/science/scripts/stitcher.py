#!/usr/bin/env python3

# from __future__ import print_function

import cv2 as cv
from cvbridge import CvBridge, CvBridgeError

# modes = (cv.Stitcher_PANORAMA, cv.Stitcher_SCANS)
 
# parser = argparse.ArgumentParser(prog='stitcher.py', description='Stitching sample.')
# parser.add_argument('--mode',
#     type = int, choices = modes, default = cv.Stitcher_PANORAMA,
#     help = 'Determines configuration of stitcher. The default is `PANORAMA` (%d), '
#          'mode suitable for creating photo panoramas. Option `SCANS` (%d) is suitable '
#          'for stitching materials under affine transformation, such as scans.' % modes)
# parser.add_argument('--output', default = 'result.jpg',
#     help = 'Resulting image. The default is `result.jpg`.')
# parser.add_argument('img', nargs='+', help = 'input images')
 
# __doc__ += '\n' + parser.format_help()
class Stitcher:
    def __init__(self, panaroma = True):
        self.bridge = CvBridge()
        if panaroma:
            self.mode = cv.Stitcher_PANORAMA
        else:
            self.mode = cv.Stitcher_SCANS

    def stitch(self, img_msgs):
        # args = parser.parse_args()
    
        # read input images
        imgs = []
        # imgfiles = ['1.jpg', '2.jpg', '3.jpg', '4.jpg', '5.jpg']
        for img_msg in img_msgs:
            # img = cv.imread(cv.samples.findFile(img_name))
            try:
                img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            if img is None:
                print("can't read image " + img_msg.frame_id + "with timestamp " + img_msg.header.stamp)
                return
                # sys.exit(-1)
            imgs.append(img)
    
        #![stitching]
        # stitcher = cv.Stitcher.create(args.mode)
        stitcher = cv.Stitcher.create(self.mode)
        status, pano = stitcher.stitch(imgs)
    
        if status != cv.Stitcher_OK:
            print("Can't stitch images, error code = %d" % status)
            return
            # sys.exit(-1)
        #![stitching]
    
        cv.imwrite("result.jpg", pano)
        print("stitching completed successfully, file saved!")
    
        print('Done')
 
 
# if __name__ == '__main__':
#     print(__doc__)
#     stitch()
#     cv.destroyAllWindows()