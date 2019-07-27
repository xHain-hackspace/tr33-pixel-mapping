import socket
import queue, threading
import time
import random
import cv2
import numpy as np
from itertools import chain
from datetime import datetime

#camera to use
WEBCAM_INDEX = 0#0 first device, 2 second device

#net settings
UDP_IP = "tr33"
UDP_PORT = 1337
COMMAND_ACK_TIMEOUT = 0.5 #seconds
COMMAND_ACK_BYTE = bytes([42])
NR_OF_COMMAND_SEND_TRIES = 10

# strip and pixel ranges to cover
TRUNK_STRIP_RANGE  = range(0,8)
TRUNK_PIXEL_RANGE  = range(0,50)
BRANCH_STRIP_RANGE = chain(range(8,12),range(13,17),range(18,20))#skip strips 12,17
BRANCH_PIXEL_RANGE = range(0,90)

#detection settings
DELAY_AFTER_LED_ON_COMMAND = 0.13# seconds
GAUSSIAN_BLUR_SIZE = 11
NR_OF_LED_DETECTION_TRIES = 3 #number of tries to detect an LED before giving up
THRESHOLDING_PERCENTAGE = 0.0005 #as a factor, not in actual percent

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

detector = cv2.SimpleBlobDetector_create(params)#finally, create detector to be used later in functions


#tree command constants
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

#exception class for failed tr33 commmands
class Tr33CommandFailedException(Exception):
	pass

# bufferless VideoCapture, stolen from Ulrich Stern's post (stackoverflow thread)
class BufferlessVideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

#tree control functions
def send_command(command): 
	remaining_tries = NR_OF_COMMAND_SEND_TRIES
	while (remaining_tries > 0):
		remaining_tries -= 1
		if send_bytes_with_ack(command): return #send was succesful, return
		print("Sending command failed, retrying")
	raise Tr33CommandFailedException #if we exceeded the number of tries raise a custom exception

def send_bytes_with_ack(bytes_to_send):
	sock.settimeout(COMMAND_ACK_TIMEOUT)#set timeout             
	sock.sendto(bytes_to_send, (UDP_IP, UDP_PORT))#send data
	try:
		response_byte = sock.recv(1)#block until ack byte is received or timeout error is raised
		if response_byte == COMMAND_ACK_BYTE:#check response
			#print("Response ACK OK.")
			return True
		else:
			print("Response ACK Wrong.")
			return False
	except socket.timeout:#catch timeout exception
		print("Timeout occured while waiting for command ACK.")
		return False
			
	
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

def averaged_frame(frames_to_average):
	(rAvg, gAvg, bAvg) = (None, None, None)
	for frame_nr in range(0, frames_to_average):
		frame = vc.read()#read next frame
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
		
def detect_led_pixel(strip,pixel):	
			#start_time = time.time()		
			set_pixel_rgb_stream(strip,pixel,255,255,255)#set target led on
			#TODO: get rid of the following delay by proper ackknowedgement implementation on tr33				
			time.sleep(DELAY_AFTER_LED_ON_COMMAND)#additional delay for esp32 to actually update the pixel
			#print("Setting LED took "+str(int(1000*(time.time()-start_time)))+" ms")

			#start_time = time.time()+str(int(1000*
			current_frame = vc.read();
			set_pixel_rgb_stream(strip,pixel,0,0,0) # switch target led off
			current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY);#bw conversion
			blurred_frame = cv2.GaussianBlur(current_frame,(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT)
			diff_frame = cv2.absdiff(blurred_frame, ref_frame)#remove reference
			#print("Getting frame and difference calculation took "+str(int(1000*(time.time()-start_time)))+" ms")
			
			#start_time = time.time()
			#calculate threshold
			hi_percentage = THRESHOLDING_PERCENTAGE # we want the hi_percentage brightest pixels
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
			#print("Thresholding took "+str(int(1000*(time.time()-start_time)))+" ms")
			thresholded_frame_blurred = cv2.GaussianBlur(thresholded_frame,(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT)
			
			#start_time = time.time()
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
			#print("Keypoint detection took "+str(int(1000*(time.time()-start_time)))+" ms")
			
			#start_time = time.time()
			im_with_keypoints_highlight = cv2.drawKeypoints(im_with_all_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			live_with_keypoints =  cv2.drawKeypoints(current_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			live_with_keypoints = cv2.drawKeypoints(live_with_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			diff_with_keypoints =  cv2.drawKeypoints(diff_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			diff_with_keypoints = cv2.drawKeypoints(diff_with_keypoints, main_keypoint, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
					
			
			
			cv2.imshow("live_view", live_with_keypoints.astype("uint8"))
			cv2.imshow("difference_image", diff_with_keypoints.astype("uint8"))
			cv2.imshow("thresholded_image", im_with_keypoints_highlight.astype("uint8"))
							
			cv2.waitKey(1)#updates images, waits for 1 ms
			#print("Preview update took "+str(int(1000*(time.time()-start_time)))+" ms")
			nr_of_points =len(keypoints)
			if (nr_of_points == 0):
				return nr_of_points, -1, -1  #return nr of points, -1 to indicate failure to find anything
			else:
				return nr_of_points, keypoints[0].pt[0], keypoints[0].pt[1] #return nr of points, x and y coordinate of main point
	
	
#start main code

#set up udp
print ("UDP target IP:", UDP_IP)
print ("UDP target port:", UDP_PORT)
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

#blink and reset tree pixels
text_file = open("mapping.h", "w")
text_file.write("//Mapping started on "+datetime.now().strftime("%d.%m.%Y, %H:%M:%S")+"\n")
text_file.write("static float mapping[][4] = {\n")
print("[INFO] setting initial tree configuration...")
update_settings(PALETTE_RAINBOW,COLORTEMP_NONE,MODE_COMMANDS)
set_color(0,STRIP_ALL,80,2)
time.sleep(0.1)
disable_all()	
update_settings(PALETTE_RAINBOW,COLORTEMP_NONE,MODE_STREAM)

cv2.namedWindow("live_view")
cv2.namedWindow("difference_image")
cv2.namedWindow("thresholded_image")
print("[INFO] opening video hardware...")
vc = BufferlessVideoCapture(WEBCAM_INDEX)

#live view for setting up
print("[INFO] set up camera, press s to acquire reference and start mapping")
while True:
	
	live_frame = vc.read(); #get new live frame
	live_frame = cv2.cvtColor(live_frame, cv2.COLOR_BGR2GRAY);
	cv2.imshow("live_view", live_frame)	
	
	key = cv2.waitKey(20)
	if key == ord('s'): # exit live view on s key
		break

#get reference frame
print("[INFO] acquiring reference frame")
for i in range (0,5):
	ref_frame_raw = vc.read();
	ref_frame = cv2.GaussianBlur(cv2.cvtColor(ref_frame_raw, cv2.COLOR_BGR2GRAY),(GAUSSIAN_BLUR_SIZE,GAUSSIAN_BLUR_SIZE),cv2.BORDER_DEFAULT);

#map pixels
x_min = None
x_max = None
y_min = None
y_max = None
mapping_size = 0

for current_section in ["branches","trunk"]:
	if current_section == "branches":
		strip_range = BRANCH_STRIP_RANGE
		pixel_range = BRANCH_PIXEL_RANGE
	elif current_section == "trunk":
		strip_range = TRUNK_STRIP_RANGE
		pixel_range = TRUNK_PIXEL_RANGE

	for current_strip in strip_range:
		for current_pixel in pixel_range:
			tries = 0
			while tries < NR_OF_LED_DETECTION_TRIES:
				tries = tries + 1
				#start_time = time.time()
				nr_of_points, x_pos, y_pos = detect_led_pixel(current_strip,current_pixel)					
				#print("Detection took "+str(int(1000*(time.time()-start_time)))+" ms")
				if nr_of_points == 0:
					print("No point detected. Trying again...")
				else:
					print ("section "+current_section+", strip "+str(current_strip)+", pixel "+str(current_pixel)+": "+str(nr_of_points) + " points detetected, main point at "+str(x_pos)+", " + str(y_pos))
					
					#write out the data to the file
					if (mapping_size != 0):#skip the comma for the very first entry, always use it for all other entrys
						text_file.write(",\n")
					text_file.write("{"+str(current_strip)+","+str(current_pixel)+","+str(x_pos)+"," + str(y_pos)+"}")
					
					#update min/max and size for detected pixels
					if (mapping_size == 0):#set min/max to the values of the very first pixel
						x_min = x_pos
						x_max = x_pos
						y_min = y_pos
						y_max = y_pos
					else: #or else update min/max normally
						if x_pos < x_min: x_min = x_pos
						if x_pos > x_max: x_max = x_pos
						if y_pos < y_min: y_min = y_pos
						if y_pos > y_max: y_max = y_pos
					mapping_size += 1
					break#break out of retry loop

#exit	
cv2.destroyWindow("live_view")
cv2.destroyWindow("difference_image")
cv2.destroyWindow("thresholded_image")
#write file footer content
text_file.write("};\n")
text_file.write("#define X_MIN "+str(x_min)+"\n")
text_file.write("#define X_MAX "+str(x_max)+"\n")
text_file.write("#define Y_MIN "+str(y_min)+"\n")
text_file.write("#define Y_MAX "+str(y_max)+"\n")
text_file.write("#define MAPPING_SIZE "+str(mapping_size)+"\n")
text_file.write("//Mapping finished on "+datetime.now().strftime("%d.%m.%Y, %H:%M:%S")+"\n")
text_file.close()
print("[INFO] exiting...")

