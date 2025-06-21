package pedroPathing.Robot.Camera.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HoughColorPipeline extends OpenCvPipeline {

    // Visualization colors
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    // Block attributes
    String dominantColor = "Unknown";
    String orientation = "Unknown";

    @Override
    public Mat processFrame(Mat input) {
        // Define cropping region
        int cropWidth = 60;  // Width of the cropped region
        int cropHeight = 40; // Height of the cropped region

        // Forward offset for camera position
        int forwardOffsetPixels = 16; // Adjust for 1-inch offset -> 2 in = 33 and 1.5 in = 25
        int cropStartY = ((input.height() - cropHeight) / 2) - forwardOffsetPixels;
        int leftOffsetPixels = 16; // Shift ROI 16 pixels to the left
        int cropStartX = Math.max(0, ((input.width() - cropWidth) / 2) - leftOffsetPixels);


        // Ensure cropping stays within bounds
        cropStartY = Math.max(0, Math.min(cropStartY, input.height() - cropHeight));
        Rect cropRect = new Rect(cropStartX, cropStartY, cropWidth, cropHeight);
        Mat croppedField = input.submat(cropRect);

        // Convert to grayscale for edge detection
        Mat gray = new Mat();
        Imgproc.cvtColor(croppedField, gray, Imgproc.COLOR_RGB2GRAY);

        // Apply Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 100, 200);

        // Apply Hough Line Transform to detect lines
        // Apply Hough Line Transform to detect lines
        Mat lines = new Mat();
        Imgproc.HoughLines(edges, lines, 1, Math.PI / 180, 100);

        int horizontalCount = 0;
        int verticalCount = 0;

// Analyze the detected lines
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            double rho = line[0];
            double theta = line[1];

            // Convert theta to degrees
            double angle = Math.toDegrees(theta);

            // Normalize angle to [0, 180]
            angle = (angle < 0) ? angle + 180 : angle;

            // Classify lines
            if ((angle >= 0 && angle <= 45) || (angle >= 135 && angle <= 180)) {
                verticalCount++;
            } else if (angle > 45 && angle < 135) {
                horizontalCount++;
            }
        }

// Normalize counts
        double total = horizontalCount + verticalCount;
        if (total > 0) {
            double verticalRatio = verticalCount / total;
            double horizontalRatio = horizontalCount / total;

            if (verticalRatio > 0.6) {
                orientation = "Vertical";
            } else {
                orientation = "Horizontal";
            }
        }
        // Perform color detection in the cropped region
        Mat hsv = new Mat();
        Imgproc.cvtColor(croppedField, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar avgHSV = Core.mean(hsv);
        dominantColor = detectColor(avgHSV);

        // Annotate the frame
        Imgproc.putText(input, "Orientation: " + orientation,
                new Point(10, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2);
        Imgproc.putText(input, "Color: " + dominantColor,
                new Point(10, 60),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2);

        // Release unused matrices
        croppedField.release();
        gray.release();
        edges.release();
        hsv.release();

        return input;
    }

    /**
     * Classify color based on average HSV values.
     */
    private String detectColor(Scalar avgHSV) {
        double hue = avgHSV.val[0];
        if (hue < 40 || hue > 160) {
            return "Red";  // Hue values less than 40 or greater than 160 are classified as red
        } else if (hue > 40 && hue < 65) {
            return "Yellow";  // Hue values between 40 and 65 are classified as yellow
        } else if (hue > 100 && hue < 140) {
            return "Blue";  // Hue values between 100 and 140 are classified as blue
        }
        return "Unknown";  // Anything outside these ranges is classified as unknown
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public String getOrientation() {
        return orientation;
    }
}
