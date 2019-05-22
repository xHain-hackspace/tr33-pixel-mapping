import cv2

def averaged_frame(frames_to_average):
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

cv2.namedWindow("preview")
print("[INFO] opening video hardware...")
vc = cv2.VideoCapture(0)

if vc.isOpened(): #check hardware status
	print("[INFO] opened video hardware successfully")
	grabbed, frame = vc.read()
else:
	print("[ERROR] opening video hardware failed")
	raise SystemExit	

ref_frame = averaged_frame(1)

while grabbed:
	
	current_frame = averaged_frame(1) #get new live frame
	diff_frame = current_frame - ref_frame
	cv2.normalize(diff_frame,diff_frame,0,255,cv2.NORM_MINMAX)
	cv2.imshow("preview", diff_frame.astype("uint8"))	
	
	key = cv2.waitKey(20)
	if key == 27: # exit on ESC
		break
	elif key == ord('a'):
		ref_frame = averaged_frame(5) #get new reference frame by averaging
		print("[INFO] acquired new reference")	
		
	#grabbed, frame = vc.read()#read next frame
	
cv2.destroyWindow("preview")
print("[INFO] exiting...")
