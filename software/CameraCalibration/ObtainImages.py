import cv2

__author__ = 'def'

def main():
    filePrefix = 'calibration_image_'

    webcam = cv2.VideoCapture()
    webcam.open(1)



    for i in range(14):
        while True:
            dummy, frame = webcam.read()

            cv2.imshow("Image: %d" %i, frame)
            k = cv2.waitKey(30) & 0xFF
            if k == ord('q'):
                return
            elif k == ord('c'):
                cv2.imwrite(filePrefix+str(i)+'.png', frame)
                break

        cv2.destroyAllWindows()

    while True:
        dummy, frame = webcam.read()

        cv2.imshow("Image extra", frame)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            return
        elif k == ord('c'):
            cv2.imwrite(filePrefix+'extra' + '.png', frame)
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()