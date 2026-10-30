import cv2

# Open the default camera
cap = cv2.VideoCapture(0)

detector = cv2.QRCodeDetector()

print("QR detector started. Press 'q' to quit.")

last_qr = ""

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Cannot access camera")
        break

    # Detect and decode QR code
    data, points, _ = detector.detectAndDecode(frame)

    if points is not None:
        points = points.astype(int).reshape(-1, 2)

        # Draw a box around the QR code
        for i in range(len(points)):
            p1 = tuple(points[i])
            p2 = tuple(points[(i + 1) % len(points)])
            cv2.line(frame, p1, p2, (0, 255, 0), 3)

        if data:
            cv2.putText(
                frame,
                data[:40],
                tuple(points[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # Print only when the decoded QR changes
            if data != last_qr:
                print("Decoded QR:", data)
                last_qr = data

    cv2.imshow("QR Code Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
