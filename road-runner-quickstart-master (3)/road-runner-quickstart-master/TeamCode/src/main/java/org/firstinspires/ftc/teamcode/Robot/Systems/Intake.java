package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Robot.Camera.Pipelines.CameraPipeline;
public class Intake extends SubsystemBase {
    public MultipleTelemetry telemetry;
    private final Servo lif;
    private final Servo rif;
    private final Servo rext;
    private final Servo lext;
    private final Servo claw;
    private final Servo rightdiffy;
    private final Servo leftdiffy;

    private CameraPipeline pipeline;
    private OpenCvCamera webcam;
    private boolean isCameraInitialized = false;

    public Intake(final HardwareMap hardwareMap, final String lifname, final String rifname,
                            final String lextname, final String rextname,
                            final String rightdiffyname, final String leftdiffyname,
                            final String clawname) {
        lif = hardwareMap.get(Servo.class, lifname);
        rif = hardwareMap.get(Servo.class, rifname);
        lext = hardwareMap.get(Servo.class, lextname);
        rext = hardwareMap.get(Servo.class, rextname);
        claw = hardwareMap.get(Servo.class, clawname);
        leftdiffy = hardwareMap.get(Servo.class, leftdiffyname);
        rightdiffy = hardwareMap.get(Servo.class, rightdiffyname);
    }

    public void initCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        if (isCameraInitialized) {
            telemetry.addLine("Camera already initialized.");
            telemetry.update();
            return; // Prevent reinitialization
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraPipeline(); // Assign pipeline here
        webcam.setPipeline(pipeline);

        telemetry.addLine("Initializing camera...");
        telemetry.update();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                isCameraInitialized = true;
                telemetry.addLine("Camera initialized successfully.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: %d", errorCode);
                telemetry.update();
            }
        });
    }


    public OpenCvCamera getCamera() {
        if (!isCameraInitialized) {
            throw new IllegalStateException("Camera has not been initialized yet!");
        }
        return webcam;
    }

    public CameraPipeline getPipeline() {
        if (!isCameraInitialized || pipeline == null) {
            throw new IllegalStateException("Pipeline has not been initialized yet! Ensure initCamera() has been called and completed.");
        }
        return pipeline;
    }

    public String ColorDetected(){
        return pipeline.getDominantColor();
    }


    public boolean isCameraInitialized() {
        return isCameraInitialized;
    }

    // Intake actions
    public void intakeopen() {
        claw.setPosition(0.5);
    }

    public void intakeclose() {
        claw.setPosition(0.24);
    }

    public void intakeDropGrabForSpecific(){
        lif.setPosition(0.65);
        rif.setPosition(0.35);
        SleepCode(200);
        claw.setPosition(0.22);

    }

    public void transferpose(){
        claw.setPosition(0.22);
        SleepCode(100);
        bringinintake();
        lext.setPosition(1);
        rext.setPosition(0.0);
    }
    public void bringinAuto(){
        lif.setPosition(0.9); //0.85
        rif.setPosition(0.1);//0.15

        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
        lext.setPosition(0.81);
        rext.setPosition(0.19);

    }

    public void bringinintake() {
        lif.setPosition(0.9); //0.85
        rif.setPosition(0.1);//0.15
        new WaitCommand(100);
        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
        lext.setPosition(0.75);
        rext.setPosition(0.25);
    }

    public void noSPMBringIN(){
        lif.setPosition(0.88); //0.85
        rif.setPosition(0.12);//0.15
        SleepCode(100);
        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
        lext.setPosition(0.69);
        rext.setPosition(0.31);

    }


    public void dropdownintake() {
        intakespinvertical();
        intakeopen();
        lext.setPosition(0.36);
        rext.setPosition(0.64);
        lif.setPosition(0.75);
        rif.setPosition(0.25);
    }

    public void dropdownintakeFor2() {
        intakeopen();
        lext.setPosition(0.86);
        rext.setPosition(0.14);
        //SleepCode(1000);
        lif.setPosition(0.6);
        rif.setPosition(0.4);
        SleepCode(500);

    }

    public void dropdownintakeFor1() {
        intakeopen();
        lext.setPosition(0.9);
        rext.setPosition(0.1);
        //SleepCode(1000);
        lif.setPosition(0.6);
        rif.setPosition(0.4);
        intakespinvertical();
        SleepCode(500);

    }

    public void dropdownNoExt() {
        lext.setPosition(0.7);
        rext.setPosition(0.3);
        lif.setPosition(0.72);
        rif.setPosition(0.28);
        intakeElevated();
        SleepCode(100);
    }


    public void teledropdowntograb(){
        lif.setPosition(0.68);
        rif.setPosition(0.32);
        new WaitCommand(200);
        intakeclose();
    }
    public void teleTransfer(){
        clearanceTransfer();
        claw.setPosition(0.28);
        new WaitCommand(300);
        bringinintake();
    }


    public void extendForColor(){
        intakespinvertical();
        intakeopen();
        lext.setPosition(0.36);
        rext.setPosition(0.64);
        lif.setPosition(0.9);
        rif.setPosition(0.1);

    }

    public void prefireintake(){
        intakeopen();
        bringinintake();
        //max extension value
        rext.setPosition(0.34);
        lext.setPosition(0.66);
        //SleepCode(200);
        intakespinvertical();
    }

    public void intakespinvertical() {
        leftdiffy.setPosition(0.4);
        rightdiffy.setPosition(0.6);
    }

    public void intakespinhorizontal(){
        leftdiffy.setPosition(0.0);
        rightdiffy.setPosition(0.2);
    }

    public void intakeSpin3(){
        leftdiffy.setPosition(0.5);
        rightdiffy.setPosition(0.5);
    }

    public void intakeElevated(){
        leftdiffy.setPosition(0.5);
        rightdiffy.setPosition(0.5);
    }

    public void intakeElevated2(){
        leftdiffy.setPosition(0.45);
        rightdiffy.setPosition(0.55);
    }

    public void DropDownNoEXTHastag2(){
        lext.setPosition(0.7);
        rext.setPosition(0.3);
        lif.setPosition(0.72);
        rif.setPosition(0.28);
        intakeElevated2();
        SleepCode(200);
    }

    public void clearanceTransfer(){
        lif.setPosition(0.75);//0.65
        rif.setPosition(0.25); //0.35
        SleepCode(300);
        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
    }

    public void clearanceTransferEx(){
        lext.setPosition(0.6);
        rext.setPosition(0.4);
        lif.setPosition(0.75);//0.65
        rif.setPosition(0.25); //0.35
        SleepCode(200);
        claw.setPosition(0.27);
        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
    }



    public void clearanceTransferForHorizontal(){

        lif.setPosition(0.65);//0.65
        rif.setPosition(0.35); //0.35
        //SleepCode(200);
        leftdiffy.setPosition(0.4);
        rightdiffy.setPosition(0.6);
        SleepCode(200);
        leftdiffy.setPosition(0.935); // towards 1
        rightdiffy.setPosition(0.065); // towards 0
    }

    public void SleepCode(long TargetSleep){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < TargetSleep){
            //telemetry.addData("sleep for", TargetSleep);
        }
    }

    public void dropPlusVertPlusGrab(){
        intakespinvertical();
        dropdownintakeFor2();
        intakeDropGrabForSpecific();
    }

    public void dropPlusVertPlusGrabFor1(){
        intakespinvertical();
        dropdownintakeFor1();
        intakeDropGrabForSpecific();
    }

    public void combinedTransfer(){
        clearanceTransfer();
        claw.setPosition(0.24);
        SleepCode(200);
        bringinintake();

    }
    public void combinedTransferHorizontal(){
        clearanceTransferForHorizontal();
        claw.setPosition(0.24);
        SleepCode(200);
        bringinintake();
    }
    public void pushbackforTransfer(){
        lext.setPosition(0.68);
        rext.setPosition(0.32);
    }
    public void FlipTransfer(){
        clearanceTransfer();
        lif.setPosition(0.88); //0.85
        rif.setPosition(0.12);

    }

    public void intakedrop3rd(){
        intakespinhorizontal();
        lif.setPosition(0.67);
        rif.setPosition(0.33);
        SleepCode(300);
        intakeclose();
    }

    public void intakePickUp(){
        lif.setPosition(0.7);
        rif.setPosition(0.3);
    }

    public void hoverthree(){
        lext.setPosition(0.65);
        rext.setPosition(0.35);
        lif.setPosition(0.72);
        rif.setPosition(0.28);
        intakespinhorizontal();
        SleepCode(400);


    }

    public void hoverOne(){
        lext.setPosition(0.6);
        rext.setPosition(0.4);
        lif.setPosition(0.72);
        rif.setPosition(0.28);
        intakeElevated();
        SleepCode(200);

    }

    public void teleTransferStage1(){
        clearanceTransfer();
        claw.setPosition(0.28);
    }


    public double[] IntakeServoData(){

        return new double[]{lext.getPosition(), rext.getPosition(), lif.getPosition(), rif.getPosition(), leftdiffy.getPosition(), rightdiffy.getPosition(), claw.getPosition()};

    }

}

