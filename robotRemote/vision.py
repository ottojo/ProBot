import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0, cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter().fourcc('M', 'J', 'P', 'G'))
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

arucoParams = cv.aruco.DetectorParameters_create()
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

cameraMatrix = np.array([[1383.751145071639, 0, 1104.352406306196],
                         [0, 1378.698961108324, 661.8663478952149],
                         [0, 0, 1]])
distCoeffs = np.array(
    [0.05576164538111176, -0.01567280209470638, 0.01861854359648359, -0.002446417538339936, 0.1304228902250324])


def inverseRTVec(rvec, tvec):
    """
    Takes a transformation x' = rotate(x) + t specified by rvec and tvec
    :param rvec: Rotation in vector form (use Rodrigues to convert between/to rotation matrix)
    :param tvec: Translation t
    :return: Rotation rotate2 and transformation t2 for transformation x = invrotate(x' - t) = rotate2(x') + t2
    """
    assert (tvec.shape == (3,))
    R, _ = cv.Rodrigues(rvec)
    R = cv.transpose(R)  # == invert
    invTvec = np.dot(R, -tvec)
    invRvec, _ = cv.Rodrigues(R)
    return invRvec, invTvec


def transformBetweenMarkers(baseRvec, baseTvec, markerRvec, markerTvec, v):
    """
    :param baseRvec: Rotation for base marker coordinate system
    :param baseTvec: Translation for base marker coordinate system
    :param markerRvec: Rotation for second marker coordinate system
    :param markerTvec: Translation for second marker coordinate system
    :param v: Vector to be transformed
    """
    #print(f"transforming {v} from marker system at {markerTvec} to base system at {baseTvec}")
    # Transform v into camera coordinates
    # invR, invT = inverseRTVec(markerRvec, markerTvec)  # v_G = markerRvec(v) + markerTvec
    # print(f"inverse translation for marker system is {invT}")
    r1, _ = cv.Rodrigues(markerRvec)
    # print(f"inverse rotation matrix for marker system is {r1}")
    vInCamera = np.dot(r1, v) + markerTvec
    #print(f"v in camera coordinates is {vInCamera}")

    # Transform into base coordinate system
    invBaseR, invBaseT = inverseRTVec(baseRvec, baseTvec)
    r2, _ = cv.Rodrigues(invBaseR)
    vInBase = np.dot(r2, vInCamera) + invBaseT
    return vInBase


v = np.array([1, 1, 1])  # in marker system -> 4,4,4 global, 2,2,2 in base system
firstTvec = np.array([2, 2, 2])
markerTvec = np.array([3, 3, 3])
firstRvec, _ = cv.Rodrigues(np.eye(3, 3))
markerRvec, _ = cv.Rodrigues(np.eye(3, 3))

v_new = transformBetweenMarkers(firstRvec, firstTvec, markerRvec, markerTvec, v)
print(v_new)
# exit(1)

while True:
    _, frame = cap.read()
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(frame, dictionary, parameters=arucoParams)
    if ids is not None:
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.03348, cameraMatrix, distCoeffs)
        for r, t in zip(rvecs, tvecs):
            cv.drawFrameAxes(frame, cameraMatrix, distCoeffs, r, t, 0.1)

        ids: np.ndarray = ids.squeeze()
        firstIndex = np.where(ids == 11)
        firstIndex = None if firstIndex[0].size == 0 else firstIndex[0][0]
        secondIndex = np.where(ids == 12)
        secondIndex = None if secondIndex[0].size == 0 else secondIndex[0][0]

        if firstIndex is not None and secondIndex is not None:  # and secondIndex is not None:
            firstTvec = tvecs[firstIndex].reshape((3,))
            firstRvec = rvecs[firstIndex]
            secondTvec = tvecs[secondIndex].reshape((3,))
            secondRvec = rvecs[secondIndex]
            xInFirstMarker = transformBetweenMarkers(firstRvec, firstTvec, secondRvec, secondTvec, np.array([0, 0, 0]))
            print(f"xInFirstMarker={xInFirstMarker}")

    cv.imshow("frame", frame)
    if cv.waitKey(20) & 0xFF == ord('q'):
        break
