import json
import cv2
import numpy as np 
import argparse
import re

def drawAxis(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0,0,255), 10)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 10)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255,0,0), 10)
    return img


def cameraExtrinsics(K, dist, objp, axis, criteria, Nx, Ny):
    cv2.namedWindow("Finding Extrinsics", cv2.WINDOW_KEEPRATIO | cv2.WINDOW_FULLSCREEN)
    cameraSource = 1
    cap = cv2.VideoCapture(cameraSource, cv2.CAP_DSHOW) 
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # set new dimensionns to cam object (not cap)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        frame = cv2.undistort(frame, K, dist)
        frame1 = np.copy(frame) # copying numpy array
        gray = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (Nx,Ny),None)
        if not ret:
            print("Cannot find corners, change checkerboard position...")
            continue
        corners1 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
        cv2.drawChessboardCorners(frame, (Nx,Ny), corners1, ret)
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners1, K, dist)
        if not ret:
            print("There was a problem solving Pose Estimation, change checkerboard position...")
            continue
        #R,_ = cv2.Rodrigues(rvecs)
        # Project 3D points to image plane to draw axis
        imgpts_axis, jac = cv2.projectPoints(axis, rvecs, tvecs, K, dist)
        frame = drawAxis(frame,corners1.astype(np.int32),imgpts_axis.astype(np.int32))
        cv2.imshow("Finding Extrinsics", frame)
        print("Press 'Y' to use the current coordinate system, 'N' to change it.")
        if cv2.waitKey(-1) == ord('y'):
            cv2.destroyAllWindows()
            print("Extrinsic parameters have been succefully determined.")
            return rvecs, tvecs,corners1.astype(np.int32), imgpts_axis.astype(np.int32)
def cameraExtrinsicsPhoto(K, dist, objp, axis, criteria, Nx, Ny):
    cap = cv2.VideoCapture("clear.bmp")
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # set new dimensionns to cam object (not cap)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        return 0
    frame = cv2.undistort(frame, K, dist)
    frame1 = np.copy(frame) # copying numpy array
    gray = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (Nx,Ny),None)
    if not ret:
        print("Cannot find corners, change checkerboard position...")
        return 0
    corners1 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1), criteria)
    cv2.drawChessboardCorners(frame, (Nx,Ny), corners1, ret)
    ret, rvecs, tvecs = cv2.solvePnP(objp, corners1, K, dist)
    if not ret:
        print("There was a problem solving Pose Estimation, change checkerboard position...")
        return 0
    #R,_ = cv2.Rodrigues(rvecs)
    # Project 3D points to image plane to draw axis
    imgpts_axis, jac = cv2.projectPoints(axis, rvecs, tvecs, K, dist)
    frame = drawAxis(frame,corners1.astype(np.int32),imgpts_axis.astype(np.int32))
    cv2.imshow("Finding Extrinsics", frame)
    cv2.waitKey()
    print("Extrinsic parameters have been succefully determined.")
    return rvecs, tvecs,corners1.astype(np.int32), imgpts_axis.astype(np.int32)

def parse_shape(value):
    ans = re.match(r'(\d+)[xX,]\s*(\d+)', value)
    if ans is not None:
        w,h = ans.groups()
        return int(w), int(h)
    raise argparse.ArgumentTypeError('pattern shape is in incorrect form')

template_json = '''
{
    "extrinsics": {
        "rvecs": %s,
        "tvecs": %s
    }
}
'''
template_txt = '''
rvecs = %s
tvecs = %s
'''

def array2str(arr):
    elems = ['%.20e' % e for e in np.reshape(arr, np.size(arr))]
    return '[' + ', '.join(elems) + ']'

def saveExtrinsics(saveas, rvecs, tvecs):
    extrinsics = {
        'rvecs': rvecs,
        'tvecs': tvecs,
    }
    p = (
        array2str(extrinsics['rvecs']),
        array2str(extrinsics['tvecs'])
    )

    if saveas == 'txt':
        data = template_txt % p
    elif saveas == 'json':
        data = template_json % p
    else:
        raise Exception('Unsupported output format')
    f = open("Extrinsics." + saveas.strip().lower(), 'w+')
    f.write(data)
    f.close()


def main(width, height, saveas):
    print("Welcome to the One Corner Detection Program! '\n' Place checkerboard pattern in the desired position for defining the World Frame,")
    # Loading Intrinsics
    f = open('intrinsics0.json')
    data = json.load(f)['intrinsics']
    K = np.array(data['K']).reshape(3,-1) # camera matrix
    dist = np.array(data['distortion']) # distortion coefficients
    # Generating object points
    Nx = width-1
    Ny = height-1
    objp = np.zeros((Nx*Ny,3), np.float32)
    objp[:,:2] = np.mgrid[0:Nx,0:Ny].T.reshape(-1,2)
    # Augmented reality elements to be added to the image
    axis = np.float32([[2,0,0], [0,2,0], [0,0,2]]) # axis X, Y, Z (RGB)
    # criteria for cv2.cornerSubPix fucntion
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    input("then press Enter to start finding the extrinsics ...")
    rvecs, tvecs, corners, img_axis  = cameraExtrinsicsPhoto(K, dist, objp, axis, criteria, Nx, Ny)
    saveExtrinsics(saveas, rvecs, tvecs)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Pose Estimation Program')
    parser.add_argument('--shape', metavar='WxH', required = True, type=str, help='shape of the pattern as WxH')
    parser.add_argument('--saveas', help='format of a file to save the camera extrinsics: json, txt')
    args = parser.parse_args()
    w, h = parse_shape(args.shape)
    saveas = args.saveas
    main(w,h,saveas)