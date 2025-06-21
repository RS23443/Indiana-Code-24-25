package pedroPathing.Robot.Camera.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CameraPipeline extends OpenCvPipeline {

    // Visualization colors
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    // Block attributes
    String dominantColor = "Unknown";
    String orientation = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        // Define the cropping region with forward offset
        int cropWidth = 80;
        int cropHeight = 60;
        int cropStartX = (input.width() - cropWidth) / 2;

        // Forward offset to account for the camera's position behind the claw
        // Offset for 1 inch: ~16.6 pixels
        // Offset for 1.5 inches: ~25 pixels
        // Offset for 2 inches: ~33 pixels
        int forwardOffsetPixels = 33; // Adjust based on testing
        int cropStartY = ((input.height() - cropHeight) / 2) - forwardOffsetPixels;

        // Ensure cropping does not go out of bounds
        cropStartY = Math.max(0, Math.min(cropStartY, input.height() - cropHeight));
        Rect cropRect = new Rect(cropStartX, cropStartY, cropWidth, cropHeight);
        Mat croppedField = input.submat(cropRect);

        // Optional: Draw cropped region for debugging
        Imgproc.rectangle(input,
                new Point(cropStartX, cropStartY),
                new Point(cropStartX + cropWidth, cropStartY + cropHeight),
                new Scalar(255, 255, 0), 2); // Blue rectangle for the ROI

        // Convert cropped field to grayscale for contour detection
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedField, gray, Imgproc.COLOR_RGB2GRAY);

        // Apply binary thresholding
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 100, 255, Imgproc.THRESH_BINARY);

        // Find contours in the binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double minArea = 500; // Minimum area to filter out noise
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = null;
            double largestArea = 0;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > minArea && area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                // Calculate bounding rectangle
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                double aspectRatio = (double) boundingRect.width / boundingRect.height;

                orientation = (aspectRatio > 1.33) ? "Horizontal" : "Vertical";

                // Determine dominant color
                Mat hsv = new Mat();
                Imgproc.cvtColor(croppedField, hsv, Imgproc.COLOR_RGB2HSV);
                Mat blockRegion = hsv.submat(boundingRect);
                Scalar avgHSV = Core.mean(blockRegion);
                dominantColor = detectColor(avgHSV);
                blockRegion.release();

                // Draw bounding rectangle and annotate
                Point rectStart = new Point(cropStartX + boundingRect.x, cropStartY + boundingRect.y);
                Point rectEnd = new Point(rectStart.x + boundingRect.width, rectStart.y + boundingRect.height);
                Imgproc.rectangle(input, rectStart, rectEnd, GREEN, 2);

                Imgproc.putText(input, "Color: " + dominantColor,
                        new Point(rectStart.x, rectStart.y - 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
                Imgproc.putText(input, "Orientation: " + orientation,
                        new Point(rectStart.x, rectStart.y - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, RED, 2);
            }
        }

        return input;
    }

    /**
     * Classify color based on average HSV values.
     */
    private String detectColor(Scalar avgHSV) {
        // Modify HSV ranges here if needed for better detection
        double hue = avgHSV.val[0];
        if (hue < 40 || hue > 160) {
            return "Red";
        } else if (hue > 40 && hue < 65) {
            return "Yellow";
        } else if (hue > 100 && hue < 140) {
            return "Blue";
        }
        return "Unknown";
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public String getOrientation() {
        return orientation;
    }
}
