
import cv2 as cv2
import argparse
import numpy as np
print cv2.__file__
print(cv2.__version__)

class ImageClassifier:
	def __init__(self,weightsFilePath,configFilePath,classLabelsFilePath,scaleFactor):
		with open(classLabelsFilePath, 'r') as f:
			self.classes = [line.strip() for line in f.readlines()]
		self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
		self.scaleFactor = scaleFactor
		self.trainedData = cv2.dnn.readNet(args.weights, args.config)

	def classify(self,image):      
		width = image.shape[1]
		height = image.shape[0]
		blob = cv2.dnn.blobFromImage(image, self.scaleFactor, (416,416), (0,0,0), True, crop=False)
		self.trainedData.setInput(blob)

		outs = self.trainedData.forward(self.get_output_layers())


		class_ids = []
		confidences = []
		boxes = []
		conf_threshold = 0.5
		nms_threshold = 0.4
			
		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax(scores)
				confidence = scores[class_id]
				if confidence > 0.5:
					center_x = int(detection[0] * width)
					center_y = int(detection[1] * height)
					w = int(detection[2] * width)
					h = int(detection[3] * height)
					x= center_x - w / 2
					y = center_y - h / 2
					class_ids.append(class_id)
					confidences.append(float(confidence))
					boxes.append([x, y, w, h])

		indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

		rects=[]

		for i in indices:
			i = i[0]
			box = boxes[i]
			x = box[0]
			y = box[1]
			w = box[2]
			h = box[3]
			rects.append([x,y,w,h])
			self.draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
        
		cv2.imwrite("./test.jpg",image)
		return rects

	def draw_prediction(self,img, class_id, confidence, x, y, x_plus_w, y_plus_h):

		label = str(self.classes[class_id])
		color = self.colors[class_id]
		cv2.rectangle(img, (int(x),int(y)), (int(x_plus_w),int(y_plus_h)), color, 2)
		cv2.putText(img, label, (int(x)-10,int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)    
	
	def get_output_layers(self):    
		layer_names = self.trainedData.getLayerNames()    
		output_layers = [layer_names[i[0] - 1] for i in self.trainedData.getUnconnectedOutLayers()]

		return output_layers

	def draw_prediction(self,img, class_id, confidence, x, y, x_plus_w, y_plus_h):
		label = str(self.classes[class_id])
		color = self.colors[class_id]
		cv2.rectangle(img, (int(x),int(y)), (int(x_plus_w),int(y_plus_h)), color, 2)
		cv2.putText(img, label, (int(x)-10,int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

if __name__ == "__main__":
	ap = argparse.ArgumentParser()
	ap.add_argument('-i', '--image', required=True,
                help = 'path to input image')
	ap.add_argument('-c', '--config', required=True,
                help = 'path to yolo config file')
	ap.add_argument('-w', '--weights', required=True,
                help = 'path to yolo pre-trained weights')
	ap.add_argument('-cl', '--classes', required=True,
                help = 'path to text file containing class names')
	args = ap.parse_args()

	classifer = ImageClassifier(args.weights, args.config,args.classes,0.00392)

	image = cv2.imread(args.image)
	rects = classifer.classify(image)
	print(rects)
