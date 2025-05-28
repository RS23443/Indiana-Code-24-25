package org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines; // Make sure this package matches your FTC project structure

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This OpenCvPipeline detects objects of a specific color (e.g., yellow)
 * within a defined Region of Interest (ROI) in a video stream.
 * It outputs a black and white image representing the ROI, where white rectangles
 * show detected yellow areas, and black represents areas without yellow within that ROI.
 *
 * Key Steps:
 * 1. Define an ROI (Region of Interest).
 * 2. Extract the ROI from the input camera frame.
 * 3. Convert the ROI image from RGB to HSV color space.
 * 4. Threshold the HSV image to create a binary mask for the target color.
 * 5. (Optional) Apply morphological operations to clean up the mask.
 * 6. Find contours (outlines) of the colored regions in the mask.
 * 7. Filter contours by area to remove noise.
 * 8. Draw filled white rectangles around the detected contours on a black background (sized to the ROI).
 * 9. Provide a method to access the coordinates of these detected rectangles, adjusted to be relative to the full input frame.
 */
public class ContrastAndROI extends OpenCvPipeline {

    // --- Configuration Constants ---
    public static Scalar LOWER_YELLOW_HSV = new Scalar(20, 100, 100); // Lower HSV threshold for yellow
    public static Scalar UPPER_YELLOW_HSV = new Scalar(30, 255, 255); // Upper HSV threshold for yellow
    private double minContourArea = 150.0; // Min area to be considered a detection
    private boolean enableMorphologicalOps = true;
    private Size morphKernelSize = new Size(5, 5);

    // --- Region of Interest (ROI) ---
    // This rectangle defines the specific area of the camera input to process.
    // It should be set using setInspectionROI() before the pipeline starts.
    private Rect inspectionROI = null;
    private Rect validROI = null; // The actual ROI used after clamping to frame boundaries

    // --- Internal Mats (OpenCV image containers) ---
    private Mat roiMat = new Mat();             // Stores the sub-image for the ROI
    private Mat hsvMat = new Mat();             // Stores the ROI in HSV color space
    private Mat binaryMask = new Mat();         // Stores the black and white mask after color thresholding (ROI-sized)
    private Mat hierarchy = new Mat();          // Stores contour hierarchy information
    private Mat outputFrame = new Mat();        // The final black and white image (ROI-sized)
    private Mat morphKernel;                    // Kernel for morphological operations

    // --- Detected Objects ---
    private volatile List<Rect> detectedRectangles = Collections.synchronizedList(new ArrayList<>());

    /**
     * Constructor: Initializes resources.
     */
    public ContrastAndROI() {
        if (enableMorphologicalOps) {
            morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphKernelSize);
        }
    }

    /**
     * Sets the Region of Interest (ROI) for processing.
     * Example: To process the bottom half of a 640x480 camera feed:
     * pipeline.setInspectionROI(new Rect(0, 240, 640, 240));
     * @param roi The rectangle defining the ROI in full camera frame coordinates.
     */
    public void setInspectionROI(Rect roi) {
        this.inspectionROI = roi;
        // validROI will be recalculated in processFrame based on current input frame size
    }

    /**
     * Gets the currently configured inspection ROI.
     * @return The Rect object representing the desired ROI, or null if not set.
     */
    public Rect getInspectionROI() {
        return this.inspectionROI;
    }


    @Override
    public Mat processFrame(Mat input) {
        // Ensure the input frame is not empty.
        if (input.empty()) {
            System.err.println("YellowDetectionPipeline: Input frame is empty.");
            return input;
        }

        // 1. Validate and define the working ROI for this frame.
        if (inspectionROI == null || inspectionROI.area() == 0) {
            System.err.println("YellowDetectionPipeline: Inspection ROI not set or has zero area. Returning black frame.");
            // Create a black output frame of the input size if not already done or if sizes differ.
            if (outputFrame.empty() || outputFrame.cols() != input.cols() || outputFrame.rows() != input.rows()) {
                outputFrame = new Mat(input.size(), CvType.CV_8UC1, Scalar.all(0));
            } else {
                outputFrame.setTo(Scalar.all(0)); // Clear to black
            }
            synchronized (detectedRectangles) {
                detectedRectangles.clear();
            }
            return outputFrame; // Return a black frame matching input size
        }

        // Clamp the user-defined ROI to be within the bounds of the input frame.
        // This prevents errors if the ROI is defined outside the actual image.
        validROI = new Rect(
                Math.max(0, inspectionROI.x),
                Math.max(0, inspectionROI.y),
                Math.min(input.cols() - Math.max(0, inspectionROI.x), inspectionROI.width),
                Math.min(input.rows() - Math.max(0, inspectionROI.y), inspectionROI.height)
        );

        if (validROI.width <= 0 || validROI.height <= 0) {
            System.err.println("YellowDetectionPipeline: Inspection ROI is outside of frame or invalid after clamping. Returning black frame.");
            if (outputFrame.empty() || outputFrame.cols() != input.cols() || outputFrame.rows() != input.rows()) {
                outputFrame = new Mat(input.size(), CvType.CV_8UC1, Scalar.all(0));
            } else {
                outputFrame.setTo(Scalar.all(0));
            }
            synchronized (detectedRectangles) {
                detectedRectangles.clear();
            }
            return outputFrame; // Return a black frame matching input size
        }

        // 2. Extract the ROI from the input image.
        // All subsequent operations will be performed on this 'roiMat'.
        // .submat() creates a header for the sub-region; data is shared with 'input' unless cloned.
        // For read-only operations on roiMat leading to new Mats (hsvMat, binaryMask), this is fine.
        roiMat = input.submat(validROI);

        // 3. Convert the ROI from RGB to HSV color space.
        Imgproc.cvtColor(roiMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // 4. Create a binary mask by thresholding the HSV image (ROI) for the yellow color range.
        Core.inRange(hsvMat, LOWER_YELLOW_HSV, UPPER_YELLOW_HSV, binaryMask);

        // 5. (Optional) Apply morphological operations to the binary mask (ROI).
        if (enableMorphologicalOps && morphKernel != null) {
            Imgproc.morphologyEx(binaryMask, binaryMask, Imgproc.MORPH_OPEN, morphKernel);
        }

        // 6. Find contours in the binary mask (ROI).
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(binaryMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // 7. Prepare the output frame. It will be the size of the ROI and initially black.
        if (outputFrame.empty() || outputFrame.cols() != roiMat.cols() || outputFrame.rows() != roiMat.rows()) {
            outputFrame = new Mat(roiMat.size(), CvType.CV_8UC1, Scalar.all(0));
        } else {
            outputFrame.setTo(Scalar.all(0)); // Clear to black for this new frame
        }

        List<Rect> currentFrameRects = new ArrayList<>();

        // 8. Iterate through detected contours found within the ROI.
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= minContourArea) {
                // Get the bounding rectangle for the contour *within the ROI's coordinate system*.
                Rect boundingRectangleInROI = Imgproc.boundingRect(contour);

                // Create a new rectangle with coordinates adjusted to be relative to the *full input frame*.
                Rect boundingRectangleInFullFrame = new Rect(
                        boundingRectangleInROI.x + validROI.x, // Add ROI's top-left X
                        boundingRectangleInROI.y + validROI.y, // Add ROI's top-left Y
                        boundingRectangleInROI.width,
                        boundingRectangleInROI.height
                );
                currentFrameRects.add(boundingRectangleInFullFrame);

                // Draw a filled white rectangle on the outputFrame (which is ROI-sized).
                // Use the ROI-relative coordinates for drawing on this outputFrame.
                Imgproc.rectangle(outputFrame, boundingRectangleInROI.tl(), boundingRectangleInROI.br(), new Scalar(255), Core.FILLED);
            }
        }

        // 9. Update the shared list of detected rectangles (with full frame coordinates).
        synchronized (detectedRectangles) {
            detectedRectangles.clear();
            detectedRectangles.addAll(currentFrameRects);
        }

        // Return the processed outputFrame (this is the black & white view of the ROI).
        return outputFrame;
    }

    /**
     * Returns a synchronized list of bounding rectangles of detected yellow objects
     * from the last processed frame. Coordinates are relative to the full input frame.
     * @return A new {@code List<Rect>} containing the detected rectangles. Empty if none found.
     */
    public List<Rect> getDetectedRectangles() {
        synchronized (detectedRectangles) {
            return new ArrayList<>(detectedRectangles); // Return a copy
        }
    }

    // --- Configuration Methods ---
    public static void setLowerYellowHsv(double h, double s, double v) {
        LOWER_YELLOW_HSV = new Scalar(h, s, v);
    }
    public static void setUpperYellowHsv(double h, double s, double v) {
        UPPER_YELLOW_HSV = new Scalar(h, s, v);
    }
    public void setMinContourArea(double area) {
        this.minContourArea = area;
    }
    public void setEnableMorphologicalOps(boolean enable) {
        this.enableMorphologicalOps = enable;
        if (enable) {
            if (morphKernel == null) {
                morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphKernelSize);
            }
        } else {
            if (morphKernel != null) {
                morphKernel.release();
                morphKernel = null;
            }
        }
    }
    public void setMorphKernelSize(double width, double height) {
        this.morphKernelSize = new Size(width, height);
        if (enableMorphologicalOps) {
            if (morphKernel != null) morphKernel.release();
            morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphKernelSize);
        }
    }

    /**
     * Call this method when the pipeline is no longer needed to free OpenCV Mat resources.
     */
    public void cleanup() {
        System.out.println("YellowDetectionPipeline: Releasing Mat resources.");
        if (roiMat != null) roiMat.release();
        if (hsvMat != null) hsvMat.release();
        if (binaryMask != null) binaryMask.release();
        if (hierarchy != null) hierarchy.release();
        if (outputFrame != null) outputFrame.release();
        if (morphKernel != null) {
            morphKernel.release();
            morphKernel = null;
        }
        synchronized (detectedRectangles) {
            detectedRectangles.clear();
        }
    }
}
