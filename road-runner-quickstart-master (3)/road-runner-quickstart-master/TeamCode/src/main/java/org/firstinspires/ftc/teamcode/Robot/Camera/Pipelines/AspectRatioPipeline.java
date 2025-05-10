package org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AspectRatioPipeline extends OpenCvPipeline {

    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    private String orientation = "Unknown";
    private String dominantColor = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        // Define ROI
        int cropWidth = 50;
        int cropHeight = 50;
        int leftOffset = 16;
        int forwardOffset = 33;

        int cropStartX = ((input.width() - cropWidth) / 2) - leftOffset;
        int cropStartY = ((input.height() - cropHeight) / 2) - forwardOffset;

        cropStartX = Math.max(0, Math.min(cropStartX, input.width() - cropWidth));
        cropStartY = Math.max(0, Math.min(cropStartY, input.height() - cropHeight));

        Rect cropRect = new Rect(cropStartX, cropStartY, cropWidth, cropHeight);
        Mat croppedField = input.submat(cropRect);

        // Preprocessing
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedField, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(binary, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint largestContour = null;
            double largestArea = 0;

            // Find the largest contour
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                // Determine orientation based on aspect ratio
                if (aspectRatio > 1.33) {
                    orientation = "Horizontal";
                } else if (aspectRatio < 1.3) {
                    orientation = "Vertical";
                } else {
                    orientation = "Square/Undetermined";
                }

                // Annotate the frame
                Imgproc.putText(input, "Orientation: " + orientation,
                        new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2);

                // Visualize bounding box
                Point rectStart = new Point(cropStartX + boundingRect.x, cropStartY + boundingRect.y);
                Point rectEnd = new Point(rectStart.x + boundingRect.width, rectStart.y + boundingRect.height);
                Imgproc.rectangle(input, rectStart, rectEnd, GREEN, 2);

                // Color Detection
                Mat hsv = new Mat();
                Imgproc.cvtColor(croppedField, hsv, Imgproc.COLOR_RGB2HSV);
                Scalar avgHSV = Core.mean(hsv);
                dominantColor = detectColor(avgHSV);

                // Annotate with color
                Imgproc.putText(input, "Color: " + dominantColor,
                        new Point(10, 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2);
            }
        }

        return input;
    }

    private String detectColor(Scalar avgHSV) {
        double hue = avgHSV.val[0];
        if (hue < 40 || hue > 160) {
            return "Red";
        } else if (hue > 40 && hue < 65) {
            return "Yellow";
        } else if (hue > 115 && hue < 140) {
            return "Blue";
        }
        return "Unknown";
    }

    public String getOrientation() {
        return orientation;
    }

    public String getDominantColor() {
        return dominantColor;
    }
}
