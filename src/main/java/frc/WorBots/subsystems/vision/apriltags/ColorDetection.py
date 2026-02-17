public class ColorDetection {
  #Modified from geeksforgeeks.org

  # Python code for Multiple Color Detection 


import numpy as np 
import cv2 


# Capturing video through webcam 
webcam = cv2.VideoCapture(0) 

# Start a while loop 
while(1): 
	
	# Reading the video from the 
	# webcam in image frames 
	_, imageFrame = webcam.read() 

	# Convert the imageFrame in 
	# BGR(RGB color space) to 
	# HSV(hue-saturation-value) 
	# color space 
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

	# Set range for yellow color and 
	# define mask 
	fuel_lower = np.array([136, 87, 111], np.uint8) 
	fuel_upper = np.array([180, 255, 255], np.uint8) 
	fuel_mask = cv2.inRange(hsvFrame, fuel_lower, fuel_upper) 

	
	# Morphological Transform, Dilation 
	# for each color and bitwise_and operator 
	# between imageFrame and mask determines 
	# to detect only that particular color 
	kernel = np.ones((5, 5), "uint8") 
	
	# For yellow color 
	fuel_mask = cv2.dilate(fuel_mask, kernel) 
	res_fuel = cv2.bitwise_and(imageFrame, imageFrame, 
							mask = fuel_mask) 
	

	# Creating contour to track yellow color 
	contours, hierarchy = cv2.findContours(fuel_mask, 
										cv2.RETR_TREE, 
										cv2.CHAIN_APPROX_SIMPLE) 
	
	for pic, contour in enumerate(contours): 
		area = cv2.contourArea(contour) 
		if(area > 300): 
			x, y, w, h = cv2.boundingRect(contour) 
			imageFrame = cv2.rectangle(imageFrame, (x, y), 
									(x + w, y + h), 
									(0, 0, 255), 2) 
			
			cv2.putText(imageFrame, "Yellow Colour", (x, y), 
						cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
						(0, 0, 255))	 


			
	# Program Termination 
	cv2.imshow("Fuel Detection in Real-TIme", imageFrame) 
	if cv2.waitKey(10) & 0xFF == ord('q'): 
		webcam.release() 
		cv2.destroyAllWindows() 
		break
}
