import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path

parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
args = parser.parse_args()
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
# Check if the given file have bag extension
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    # Read feed from a bag file
    rs.config.enable_device_from_file(config, args.input, False) # Do not repeat the input

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)

    # Check the device information 
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    print(device_product_line)

    # get in right configuration
    # zdepth 
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)

    # color in rgb8 but actually what we get is in bgr space
    config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
    
    # Start streaming from file
    profile = pipeline.start(config)#, queue)
    
    # Disable realtime playback so we have more time to process the frames
    profile.get_device().as_playback().set_real_time(False)

    # Get depth scale to get it align
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth - depth Scale is: ", depth_scale)

    # For background peeling
    # clipping_distance_in_meters = 1  # 1 meter
    # clipping_distance = clipping_distance_in_meters / depth_scale


    # alignment
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create colorizer object
    colorizer = rs.colorizer()
    success = True
    frameSet = []
    iteration = 0
    
    print('Reading all frames from stream...')
    
    # Streaming loop
    while True:
        # Get frameset of depth
        s, frames = pipeline.try_wait_for_frames()
        if not s:
            print('End of stream')
            break
        
        frame_number = frames.get_frame_number()
        
        # Get frame number and set a finite set
        if frame_number in frameSet:
            print(iteration, 'frame repeated, stopping', frame_number)
            break
        else:
            frameSet.append(frame_number)
            print(iteration, frame_number)


        aligned_frames = align.process(frames)

        # Get depth frame and rgb frame for aligned frame
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if iteration == 0:
            depth_intrinsic = depth_frame.profile.as_video_stream_profile().intrinsics
            color_intrinsic = color_frame.profile.as_video_stream_profile().intrinsics
            print("Color - color - Width", color_intrinsic.width)
            print("Depth - depth - Width", depth_intrinsic.width)
            
            print("Color - Height", color_intrinsic.height)
            print("Depth - Height", depth_intrinsic.height)

            print("Color - MX",color_intrinsic.ppx)
            print("Depth - MX",depth_intrinsic.ppx)

            print("Color - MY",color_intrinsic.ppy)
            print("Depth - MY",depth_intrinsic.ppy)

            print("Color - Fx",color_intrinsic.fx)
            print("Depth - Fx",depth_intrinsic.fx)

            print("Color - Fy",color_intrinsic.fy)
            print("Depth - Fy",depth_intrinsic.fy)

            # print(depth_intrinsic.model)
            # print(depth_intrinsic.coeffs)
            print(color_frame)
        
        # Colorize depth frame to jet colormap
        # depth_color_frame = colorizer.colorize(depth_frame)

        # Convert depth_frame to numpy array to render image in opencv
        depth_image = np.asanyarray(depth_frame.get_data())

        # depth_color_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # grey_color = 153
        # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # images = np.asanyarray(bg_removed)
        # depth_colormap_images = np.asanyarray(depth_colormap)

        # Render image in opencv window
        cv2.imshow("Depth - depth Stream", depth_image)
        cv2.imwrite("Output/depth/" + str(iteration).zfill(6) + ".png", depth_image)
        bgrImage = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("RGB Stream", bgrImage)
        cv2.imwrite("Output/rgb/" + str(iteration).zfill(6) + ".png", bgrImage)
        # cv2.imshow("Align depth", images)
        # cv2.imshow("Align-2", depth_colormap_images)
        iteration += 1
        key = cv2.waitKey(1)
        # if pressed escape exit program
        if key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass
