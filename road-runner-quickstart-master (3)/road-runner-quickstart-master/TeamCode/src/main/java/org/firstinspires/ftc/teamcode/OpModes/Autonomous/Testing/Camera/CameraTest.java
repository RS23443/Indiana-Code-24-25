package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Testing.Camera; // Adjust this package to your FTC project structure

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.ContrastAndROI; // UPDATED: Changed from YellowDetectionPipeline
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * This is an example LinearOpMode that shows how to use the ContrastAndROI pipeline.
 * It initializes a webcam, sets up the pipeline with a Region of Interest (ROI),
 * and displays telemetry about detected yellow objects.
 *
 * Make sure you have EasyOpenCV configured in your project.
 * You will also need to have the ContrastAndROI class in your project,
 * typically under a 'vision' package.
 */
@TeleOp(name = "Yellow Detection Test", group = "Concept")
// @Disabled
public class CameraTest extends LinearOpMode {

    OpenCvCamera webcam;
    ContrastAndROI yellowPipeline;

    // --- Camera Configuration ---
    // Adjust these values based on your camera and desired resolution.
    // Common resolutions: 320x240, 640x480, 1280x720
    int cameraWidth = 1280;  // pixels
    int cameraHeight = 720; // pixels

    // --- ROI Definition ---
    // Example: A rectangle in the bottom-center of the camera view.
    // You MUST adjust this ROI to suit your robot's needs and camera placement.
    // Rect(x, y, width, height)
    // x: pixels from left edge of the full frame
    // y: pixels from top edge of the full frame
    // width: width of the ROI in pixels
    // height: height of the ROI in pixels
    Rect inspectionROI; // Will be initialized in runOpMode

    // Flag to track if the camera is currently streaming
    private volatile boolean isWebcamStreaming = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- Initialize ROI ---
        // Example: Define an ROI that is a 200x100 rectangle, 50px from the bottom, centered horizontally
        int roiWidth = 1280;
        int roiHeight = 720;
        int roiX = (cameraWidth - roiWidth) / 2; // Center horizontally
        int roiY = (cameraHeight-roiHeight)/2;    // 50px from the bottom edge
        inspectionROI = new Rect(roiX, roiY, roiWidth, roiHeight);


        // --- Initialize Camera and Pipeline ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId); // "Webcam 1" is the name in your robot configuration

        yellowPipeline = new ContrastAndROI(); // UPDATED: Changed from YellowDetectionPipeline
        yellowPipeline.setInspectionROI(inspectionROI); // Set the ROI for the pipeline

        // (Optional) Adjust pipeline parameters if needed, e.g., HSV thresholds or min area
        // ContrastAndROI.setLowerYellowHsv(20, 100, 100); // Assuming static methods if they exist in ContrastAndROI
        // ContrastAndROI.setUpperYellowHsv(35, 255, 255); // Assuming static methods if they exist in ContrastAndROI
        // yellowPipeline.setMinContourArea(100);

        webcam.setPipeline(yellowPipeline);

        // --- Start Camera Streaming ---
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Configure camera streaming parameters.
                // The resolution here should match what you expect for your ROI calculations.
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
                isWebcamStreaming = true; // Set flag when streaming starts
                telemetry.addData("Status", "Webcam opened and streaming");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                isWebcamStreaming = false; // Ensure flag is false on error
                telemetry.addData("Error", "Webcam could not be opened. Error Code: " + errorCode);
                telemetry.update();
                // Consider adding logic here to prevent the OpMode from continuing if the camera fails.
            }
        });

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.addData("Pipeline ROI", "x: %d, y: %d, w: %d, h: %d",
                inspectionROI.x, inspectionROI.y, inspectionROI.width, inspectionROI.height);
        telemetry.update();

        // Wait for the OpMode to start, but also check if stop is requested during initialization.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting for start... ");
            telemetry.addData("Webcam Streaming", isWebcamStreaming); // Show streaming status
            telemetry.update();
            sleep(50); // Small delay to prevent busy-waiting
        }


        if (isStopRequested()) {
            // Clean up if stop is requested before the loop starts or during waitForStart
            if (webcam != null) {
                if (isWebcamStreaming) {
                    webcam.stopStreaming();
                    isWebcamStreaming = false;
                }
                webcam.closeCameraDevice();
            }
            telemetry.addData("Status", "OpMode Stop Requested during Init. Camera closed.");
            telemetry.update();
            return;
        }

        // --- Main Loop ---
        while (opModeIsActive() && !isStopRequested()) {
            if (!isWebcamStreaming) {
                telemetry.addData("Status", "Waiting for webcam to start streaming...");
                telemetry.update();
                sleep(50); // Wait a bit if streaming hasn't started yet
                continue;
            }

            // Get the latest detections from the pipeline.
            // getDetectedRectangles() returns a *copy* of the list, so it's thread-safe.
            List<Rect> detectedYellowObjects = yellowPipeline.getDetectedRectangles();

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Pipeline Time (ms)", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", webcam.getOverheadTimeMs());
            telemetry.addData("Total Frame Time (ms)", webcam.getTotalFrameTimeMs());
            telemetry.addLine("\n--- Yellow Detection ---");
            telemetry.addData("Objects Detected", detectedYellowObjects.size());

            if (!detectedYellowObjects.isEmpty()) {
                // Display info about the first detected object
                // The coordinates in 'rect' are relative to the full camera frame.
                Rect firstObject = detectedYellowObjects.get(0);
                telemetry.addData("  Object 1 X", firstObject.x);
                telemetry.addData("  Object 1 Y", firstObject.y);
                telemetry.addData("  Object 1 Width", firstObject.width);
                telemetry.addData("  Object 1 Height", firstObject.height);
                telemetry.addData("  Object 1 Center X", firstObject.x + firstObject.width / 2.0);
                telemetry.addData("  Object 1 Center Y", firstObject.y + firstObject.height / 2.0);

                // You can add logic here to act based on the detected objects.
                // For example, if an object is in a certain position, move the robot.
                // if (firstObject.x < cameraWidth / 3) {
                //     telemetry.addLine("  Object is on the LEFT");
                // } else if (firstObject.x + firstObject.width > cameraWidth * 2 / 3) {
                //     telemetry.addLine("  Object is on the RIGHT");
                // } else {
                //     telemetry.addLine("  Object is in the CENTER");
                // }

            } else {
                telemetry.addLine("  No yellow objects found in ROI.");
            }

            telemetry.update();

            // Allow other processes to run.
            sleep(20); // Optional: adjust sleep time as needed
        }

        // --- Cleanup ---
        // Stop streaming and close the camera when the OpMode is stopped.
        if (webcam != null) {
            if (isWebcamStreaming) { // Check our flag
                webcam.stopStreaming();
                isWebcamStreaming = false; // Update flag
            }
            // Ensure pipeline resources are released if it has a cleanup method
            if (yellowPipeline != null && yellowPipeline instanceof AutoCloseable) { // Or a specific cleanup method
                try {
                    ((AutoCloseable)yellowPipeline).close(); // Example if it implements AutoCloseable
                } catch (Exception e) {
                    telemetry.addData("Error", "Exception closing pipeline: " + e.getMessage());
                }
            } else {
                try {
                    if (yellowPipeline != null && yellowPipeline.getClass().getMethod("cleanup") != null) {
                        // If your pipeline has a specific cleanup() method like the original YellowDetectionPipeline
                        // you would call it here. For simplicity, this example assumes it might not.
                        // yellowPipeline.cleanup(); // Uncomment if your ContrastAndROI has this method
                    }
                } catch (NoSuchMethodException e) {
                    throw new RuntimeException(e);
                }
            }
            webcam.closeCameraDevice(); // Important to release the camera
        }
        telemetry.addData("Status", "OpMode Stopped. Camera closed.");
        telemetry.update();
    }
}
