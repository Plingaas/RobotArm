import cv2
import os

i = 702

while True:
    filename = os.path.join('.', 'images', f'screenshot{i}.png')


    if not os.path.exists(filename):
        break

    image = cv2.imread(filename)

    if image is not None:
        resized_image = cv2.resize(image, (640, 480))
        cv2.imwrite(filename, resized_image)
    else:
        print(f'Error: Unable to read {filename}')

    i += 1

cv2.destroyAllWindows()

#filename = os.path.join(".", "chesscomboard.png")
#image = cv2.imread(filename)
#cv2.imwrite(filename, cv2.resize(image, (640,640)))