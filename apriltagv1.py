import apriltag
import cv2
from PIL import Image
from numpy import asarray
img = Image.open('photo.jpg')
imggrey = img.convert('L')
data = asarray(imggrey)
detector = apriltag.Detector()
result = detector.detect(data)

print(result)
print(result[0][6][0])
