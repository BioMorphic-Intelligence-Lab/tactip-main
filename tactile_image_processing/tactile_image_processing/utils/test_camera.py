"""
Author: Martijn Brummelhuis
"""
import cv2

def main(source, exposure=-1):
    # Initialize the camera. 0 is typically the default camera, but you can change it if you have multiple cameras.
    cap = cv2.VideoCapture(source)

    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 2)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

    # Check if the camera is opened correctly
    if not cap.isOpened():
        print("Error: Could not open video stream or file")
        exit()

    # Continuously capture frames from the camera
    while True:
        ret, frame = cap.read()

        # If frame reading was successful
        if ret:
            # Display the resulting frame
            cv2.imshow('Camera Stream', frame)
            
            # Exit if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Error: Failed to capture frame")
            break

    # Release the camera and close any open windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(4, exposure=100)