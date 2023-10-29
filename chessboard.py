import cv2
import numpy as np

# Get the sprites from texture image
boardimg = cv2.imread("resources/chesscomboard.png")
piecesimg = cv2.imread("resources/pieces.png")
w = 132
h = 132
w_rook = piecesimg[0:h, 0:w]
w_knight = piecesimg[0:h, w:w*2]
w_bishop = piecesimg[0:h, w*2:w*3]
w_queen = piecesimg[0:h, w*3:w*4]
w_king = piecesimg[0:h, w*4:w*5]
w_pawn = piecesimg[0:h, w*5:w*6]
b_rook = piecesimg[h:h*2, 0:w]
b_knight = piecesimg[h:h*2, w:w*2]
b_bishop = piecesimg[h:h*2, w*2:w*3]
b_queen = piecesimg[h:h*2, w*3:w*4]
b_king = piecesimg[h:h*2, w*4:w*5]
b_pawn = piecesimg[h:h*2, w*5:w*6]
pieceList = [b_bishop, b_king, b_knight, b_pawn, b_queen, b_rook, w_bishop, w_king, w_knight, w_pawn, w_queen, w_rook]

#y_pos = (7-0)*h
#x_pos = 3*w
#sprite = pieceList[0]

#boardimg[y_pos:y_pos + sprite.shape[0], x_pos:x_pos + sprite.shape[1]] = sprite

#boardimg = cv2.resize(boardimg, (640, 640))
#cv2.imshow("boardimg", boardimg)
#cv2.waitKey(0)

def bottomLeft(corners):
    x1 = corners[0][0]
    x2 = corners[8][0]
    x3 = corners[16][0]

    dx1 = x2-x1
    dx2 = x3-x2

    y1 = corners[0][1]
    y2 = corners[8][1]
    y3 = corners[16][1]

    dy1 = y2-y1
    dy2 = y3-y2

    dist1 = np.sqrt(dx1**2 + dy1**2)
    dist2 = np.sqrt(dx2**2 + dy2**2)
    ratio = dist1/dist2

    x = x1 - dx1*ratio
    y = y1 - dy1*ratio
    return (int(x), int(y))

def topLeft(corners):
    x1 = corners[6][0]
    x2 = corners[12][0]
    x3 = corners[18][0]

    dx1 = x2-x1
    dx2 = x3-x2

    y1 = corners[6][1]
    y2 = corners[12][1]
    y3 = corners[18][1]

    dy1 = y2-y1
    dy2 = y3-y2

    dist1 = np.sqrt(dx1**2 + dy1**2)
    dist2 = np.sqrt(dx2**2 + dy2**2)
    ratio = dist1/dist2

    x = x1 - dx1*ratio
    y = y1 - dy1*ratio
    return (int(x), int(y))

def topRight(corners):
    x1 = corners[48][0]
    x2 = corners[40][0]
    x3 = corners[32][0]

    dx1 = x2-x1
    dx2 = x3-x2

    y1 = corners[48][1]
    y2 = corners[40][1]
    y3 = corners[32][1]

    dy1 = y2-y1
    dy2 = y3-y2

    dist1 = np.sqrt(dx1**2 + dy1**2)
    dist2 = np.sqrt(dx2**2 + dy2**2)
    ratio = dist1/dist2

    x = x1 - dx1*ratio
    y = y1 - dy1*ratio
    return (int(x), int(y))

def bottomRight(corners):
    x1 = corners[42][0]
    x2 = corners[36][0]
    x3 = corners[30][0]

    dx1 = x2-x1
    dx2 = x3-x2

    y1 = corners[42][1]
    y2 = corners[36][1]
    y3 = corners[30][1]

    dy1 = y2-y1
    dy2 = y3-y2

    dist1 = np.sqrt(dx1**2 + dy1**2)
    dist2 = np.sqrt(dx2**2 + dy2**2)
    ratio = dist1/dist2

    x = x1 - dx1*ratio
    y = y1 - dy1*ratio
    return (int(x), int(y))


def getCornerPoints(corners):

    first = corners[0]
    second = corners[1]

    dy = second[1] - first[1]
    print(dy)

    tl = topLeft(corners)
    tr = topRight(corners)
    bl = bottomLeft(corners)
    br = bottomRight(corners)

    cornerList = np.float32([tl, tr, bl, br])

    top_left = min(cornerList, key=lambda corner: corner[0] + corner[1])
    top_right = min(cornerList, key=lambda corner: -corner[0] + corner[1])
    bottom_left = min(cornerList, key=lambda corner: corner[0] - corner[1])
    bottom_right = max(cornerList, key=lambda corner: corner[0] + corner[1])

    return np.float32([top_left, top_right, bottom_left, bottom_right])

def getCornersFromImage(img):

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners = cv2.findChessboardCorners(gray_img, (7,7), None)[1]

    if corners is None or len(corners) < 49:
        return None

    new_corners = []
    for i in range(len(corners)):
        new_corners.append(np.ravel(corners[i]))

    corners = new_corners[::-1]
    return getCornerPoints(corners)

def getPerspectiveTransformationMatrix(input_points):
    width = 640
    height = 640
    converted_points = np.float32([[0,0], [width,0], [0,height],[width,height]])
    return cv2.getPerspectiveTransform(input_points, converted_points)

def extractBoard(img, input_points):
    width = 640
    height = 640
    matrix = getPerspectiveTransformationMatrix(input_points)
    img_output = cv2.warpPerspective(img, matrix, (width, height))

    return img_output

def chessboardFromImage(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = cv2.findChessboardCorners(gray_img, (7,7), None)[1]

    if corners is None or len(corners) < 49:
        return None

    new_corners = []
    for i in range(len(corners)):
        new_corners.append(np.ravel(corners[i]))

    corners = new_corners[::-1]


    cornerPoints = getCornerPoints(corners)

    return extractBoard(img, cornerPoints)

# Applies a projection transformation matrix to a point and returns the new transformed point.
def transformPoint(p, matrix):
    px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    p_after = (int(px), int(py))
    return p_after

def getSquareFromPoint(p):
    x = p[0]
    y = 640-p[1] # Offset by height = 640 to have origin be bottom left

    x_square = int(x/80)
    y_square = int(y/80)

    return (x_square, y_square)