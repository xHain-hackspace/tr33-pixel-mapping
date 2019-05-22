import socket
import time
import random
import cv2
import numpy as np

GAUSSIAN_BLUR_SIZE = 11

UDP_IP = "192.168.42.189"
UDP_PORT = 1337

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

COMMAND_DISABLE = 0
COMMAND_COLOR = 1
COMMAND_UPDATE_SETTINGS = 101
COMMAND_PIXEL = 103
COMMAND_PIXEL_RGB = 104

MODE_COMMANDS = 0
MODE_STREAM = 1

PALETTE_RAINBOW = 0
PALETTE_FOREST = 1

COLORTEMP_NONE = 0

STRIP_ALL = 22

#tree control functions

def send_command(command):                     
	sock.sendto(command, (UDP_IP, UDP_PORT))
	
def set_color(index,strip,hue,brightness):
	command_bytes = bytearray()
	command_bytes.append(index)
	command_bytes.append(COMMAND_COLOR)
	command_bytes.append(strip)
	command_bytes.append(hue)
	command_bytes.append(brightness)
	send_command(command_bytes)
	
def update_settings(palette,color_temp, display_mode):
	command_bytes = bytearray()
	command_bytes.append(0)
	command_bytes.append(COMMAND_UPDATE_SETTINGS)
	command_bytes.append(palette)
	command_bytes.append(color_temp)
	command_bytes.append(display_mode)
	send_command(command_bytes)	
	
def set_pixel_stream(strip,pixel, hue):
	command_bytes = bytearray()
	command_bytes.append(0)
	command_bytes.append(COMMAND_PIXEL)
	command_bytes.append(strip)
	command_bytes.append(pixel)
	command_bytes.append(hue)
	send_command(command_bytes)	
	
def set_pixel_rgb_stream(strip,pixel, red, green, blue):
	command_bytes = bytearray()
	command_bytes.append(0)
	command_bytes.append(COMMAND_PIXEL_RGB)
	command_bytes.append(strip)
	command_bytes.append(pixel)
	command_bytes.append(red)
	command_bytes.append(green)
	command_bytes.append(blue)
	send_command(command_bytes)	
	
def disable_all():
	for index in range(0,10):
		command_bytes = bytearray()
		command_bytes.append(index)
		command_bytes.append(COMMAND_DISABLE)
		send_command(command_bytes)
		
#camera functions

def flush_frames():
	for i in range(0,6):#hack to flush the read buffer
			vc.read()

def averaged_frame(frames_to_average):
	flush_frames()
	(rAvg, gAvg, bAvg) = (None, None, None)
	for frame_nr in range(0, frames_to_average):
		grabbed, frame = vc.read()#read next frame
		(B, G, R) = cv2.split(frame.astype("float"))#split to components   
	
		# if the frame averages are None, initialize them
		if rAvg is None:
			rAvg = R
			bAvg = B
			gAvg = G
	 
		# otherwise, compute the weighted average between the history of
		# frames and the current frames
		else:
			rAvg = ((frame_nr * rAvg) + (1 * R)) / (frame_nr + 1.0)
			gAvg = ((frame_nr * gAvg) + (1 * G)) / (frame_nr + 1.0)
			bAvg = ((frame_nr * bAvg) + (1 * B)) / (frame_nr + 1.0)
	
	avg = cv2.merge([bAvg, gAvg, rAvg]).astype("float")
	return avg
	
#set up detector for use in led detetction
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
 
# Change thresholds
#params.minThreshold = 10;
#params.maxThreshold = 200;
 
# Filter by Area.
params.filterByArea = True
params.minArea = 20
params.maxArea = 2000
 
# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1
 
# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87
 
# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.01

params.minRepeatability = 2

detector = cv2.SimpleBlobDetector_create(params) #cv2.SimpleBlobDetector()	
	
def detect_led_pixel(strip,pixel):
			
			#get ref frame (can be dropped if lighting is stable long-term)
			#~ flush_frames()
			#~ grabbed, ref_frame_raw = vc.read();
			#~ ref_frame = cv2.GaussianBlur(cv2.cvtColor(ref_frame_raw, cv2.COLOR_BGR2GRAY),(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT);
			
			set_pixel_rgb_stream(strip,pixel,255,255,255)
				
			flush_frames()
			grabbed,current_frame = vc.read();
			current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY);#bw conversion
			blurred_frame = cv2.GaussianBlur(current_frame,(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT)
			diff_frame = cv2.absdiff(blurred_frame, ref_frame)#remove reference
			
			#calculate threshold
			hi_percentage = 0.0005 # we want the hi_percentage brightest pixels
			# * histogram
			hist = cv2.calcHist([diff_frame], [0], None, [256], [0, 256]).flatten()
			# * find brightness threshold
			# here: highest thresh for including at least soandso image pixels
			total_count = diff_frame.shape[0] * diff_frame.shape[1]  # height * width
			target_count = hi_percentage * total_count # bright pixels we look for
			summed = 0
			for i in range(255, 0, -1):
				summed += int(hist[i])
				if target_count <= summed:
					hi_thresh = i
					break
			else:
				hi_thresh = 0	
					
			thr,thresholded_frame = cv2.threshold(diff_frame, hi_thresh, 255, cv2.THRESH_BINARY_INV);#apply threshold	
			thresholded_frame_blurred = cv2.GaussianBlur(thresholded_frame,(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT)
			keypoints = detector.detect(thresholded_frame_blurred)#detect blobs
			im_with_all_keypoints = cv2.drawKeypoints(thresholded_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			#select top candidate for highlighting based on intensity in center
			main_keypoint_intensity = 0
			main_keypoint_index = 0			
			if (len(keypoints) != 0):
				for keypoint_index in range(0,len(keypoints)):
					current_intensity = blurred_frame[int(keypoints[keypoint_index].pt[1])][int(keypoints[keypoint_index].pt[0])]
					if (current_intensity > main_keypoint_intensity):
						main_keypoint_intensity = current_intensity
						main_keypoint_index = keypoint_index
						
			main_keypoint = keypoints[main_keypoint_index:(main_keypoint_index+1)]
			
			
			im_with_keypoints_highlight = cv2.drawKeypoints(im_with_all_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			live_with_keypoints =  cv2.drawKeypoints(current_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			live_with_keypoints = cv2.drawKeypoints(live_with_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			diff_with_keypoints =  cv2.drawKeypoints(diff_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			diff_with_keypoints = cv2.drawKeypoints(diff_with_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
					
			set_pixel_rgb_stream(strip,pixel,0,0,0)
			set_pixel_rgb_stream(strip,pixel,0,0,0)
			set_pixel_rgb_stream(strip,pixel,0,0,0)
			time.sleep(0.2)
			
			cv2.imshow("preview", live_with_keypoints.astype("uint8"))
			cv2.imshow("preview2", diff_with_keypoints.astype("uint8"))
			cv2.imshow("preview3", im_with_keypoints_highlight.astype("uint8"))
							
			cv2.waitKey(1)
			if (len(keypoints) == 0):
				result_str=str(strip)+","+str(pixel)+",-1,-1,NO_LED_FOUND"
				print(result_str)
				text_file.write(result_str+'\n')
			elif (len(keypoints) == 1):
				result_str=str(strip)+","+str(pixel)+","+str(keypoints[0].pt[0])+","+str(keypoints[0].pt[1])+",ONE_LED_FOUND"
				print(result_str)
				text_file.write(result_str+'\n')
			else:
				result_str=str(strip)+","+str(pixel)+","+str(keypoints[0].pt[0])+","+str(keypoints[0].pt[1])+",MULTIPLE_LEDS_FOUND"
				print(result_str)
				text_file.write(result_str+'\n')
	
	
#start main code

#blink and reset tree pixels
text_file = open("Log.txt", "w")
text_file.write("***Starting new run... ***\n")
print("[INFO] setting initial tree configuration...")
update_settings(PALETTE_RAINBOW,COLORTEMP_NONE,MODE_COMMANDS)
set_color(0,STRIP_ALL,80,2)
time.sleep(0.1)
disable_all()	
update_settings(PALETTE_RAINBOW,COLORTEMP_NONE,MODE_STREAM)

cv2.namedWindow("preview")
cv2.namedWindow("preview2")
cv2.namedWindow("preview3")
print("[INFO] opening video hardware...")
vc = cv2.VideoCapture(0)

if vc.isOpened(): #check hardware status
	print("[INFO] opened video hardware successfully")
	grabbed, frame = vc.read()
else:
	print("[ERROR] opening video hardware failed")
	raise SystemExit	

#live view for setting up
print("[INFO] set up camera, press s t acquire reference and start mapping")
while grabbed:
	
	grabbed, live_frame = vc.read(); #get new live frame
	live_frame = cv2.cvtColor(live_frame, cv2.COLOR_BGR2GRAY);
	cv2.imshow("preview", live_frame)	
	
	key = cv2.waitKey(20)
	if key == ord('s'): # exit live view on s key
		break

#get reference frame
print("[INFO] acquiring reference frame")
for i in range (0,1):
	flush_frames()
	grabbed, ref_frame_raw = vc.read();
	ref_frame = cv2.GaussianBlur(cv2.cvtColor(ref_frame_raw, cv2.COLOR_BGR2GRAY),(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT);

#map pixels
while grabbed:		
	#~ for trunk in range(0,8):
		#~ for pixel in range(20,50):
			#~ detect_led_pixel(trunk,pixel)		

	for branch in range(9,12)+range(13,20):
		for pixel in range(0,90):
			detect_led_pixel(branch,pixel)

#exit	
cv2.destroyWindow("preview")
cv2.destroyWindow("preview2")
cv2.destroyWindow("preview3")
print("[INFO] exiting...")
text_file.write("Done, exiting.\n")
text_file.close()

