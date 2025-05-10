package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.Sensing.PDFL;
import org.firstinspires.ftc.teamcode.Robot.Sensing.Timer;

public class Lifts extends SubsystemBase {
    public MultipleTelemetry telemetry;

    private final DcMotorEx rightslides;
    private final DcMotorEx leftslides;
    public static double i = 0.00, p = 0.012, d = 0.000  ;
    //public static double p1 = 0.02, i1 = 0.00, d1 = 0.00;
    public static double maxticks = 1200.00;
    public static double maxTolerableTicks = 1090.00;
    private static double kG= 0.19;
    private final double ticks_in_degree = 145.1 / 180.0;

    private PDFL pdfLController;

    // Tuning variables accessible via FTC Dashboard
    public static double kP = 0.01, kD = 0.005, kF = 0.2, kL = 0.2;
    public static double deadzone = 20.0;
    public static double homedConstant = -10.0;
    public VoltageSensor controlHubVoltageSensor;
    public Timer braketimer;
    private int targetHeight;
    private boolean pdflEnabled = false;
    private long lastUpdateTime = 0;
    private final long updateInterval = 50; // Adjust this if needed



    public Lifts(final HardwareMap hardwareMap, final double voltage) {
        leftslides = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslides = hardwareMap.get(DcMotorEx.class, "rightslide");
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        kP = kP * (12/voltage);
        kD = kD * (12/voltage);
        kF = kF * (12/voltage);
        kL = kL * (12/voltage);
        pdfLController = new PDFL(kP,kD,kF,kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);
        braketimer = new Timer();
        leftslides.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void joystick(Gamepad gamepad, double speed){
        rightslides.setPower(-gamepad.left_stick_y* speed);
        leftslides.setPower(-gamepad.left_stick_y * speed);
    }
    public void slidePID(int target) {
        if(leftslides.getCurrentPosition() != target) {
            PIDController controllerleft = new PIDController(p, i, d);
            //PIDController controllerright= new PIDController(p1,i1,d1);

            int armPos = leftslides.getCurrentPosition();
            int armPos1 = rightslides.getCurrentPosition();
            double pid = controllerleft.calculate(armPos, Math.min(target, maxTolerableTicks));
            //double pid1 = controllerright.calculate(armPos1, Math.min(target,maxTolerableTicks));
            double power = pid + kG;


            leftslides.setPower(power);
            rightslides.setPower(power);
        }

    }

    public void slidePDFL(int target){
        if(leftslides.getCurrentPosition() != target){

            int leftSlidePos = leftslides.getCurrentPosition();
            int rightSlidePos = rightslides.getCurrentPosition();

            // Calculate control signals
            double error = target - leftSlidePos;
            double error1 = target - rightSlidePos;

            double powerLeft = pdfLController.run(error);
            double powerRight = pdfLController.run(error1);

            powerLeft = Math.max(-1, Math.min(powerLeft, 1));
            powerRight = Math.max(-1, Math.min(powerRight, 1));

            leftslides.setPower(powerLeft);
            rightslides.setPower(powerLeft);
        }
    }

    public void setTarget(int target) {
        this.targetHeight = target;
    }

    public void enablePDControl(boolean enable) {
        this.pdflEnabled = enable;
    }

    public double DynamicKF(){
        return kF;
    }

    public void updatePDControl() {
        if (!pdflEnabled) return;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < updateInterval) return; // Prevent rapid updates
        lastUpdateTime = currentTime; // Reset timer

        int leftSlidePos = leftslides.getCurrentPosition();
        int rightSlidePos = rightslides.getCurrentPosition();

        double errorLeft = targetHeight - leftSlidePos;
        double errorRight = targetHeight - rightSlidePos;

        double powerLeft = pdfLController.run(errorLeft);
        double powerRight = pdfLController.run(errorRight);

        powerLeft = Math.max(-1, Math.min(powerLeft, 1));
        powerRight = Math.max(-1, Math.min(powerRight, 1));

        leftslides.setPower(powerLeft);
        rightslides.setPower(powerRight);
        //if (Math.abs(errorLeft) < deadzone || Math.abs(errorRight) < deadzone) {
          //  pdflEnabled = false;  // Stop updating once the error is within the threshold
        //}

    }

    public boolean VoltageReaction(double minVoltage){
        minVoltage = minVoltage - (12/controlHubVoltageSensor.getVoltage());
        double currentVoltage = controlHubVoltageSensor.getVoltage();
        if(currentVoltage < minVoltage){
            braketimer.startTimer();
        } else{
            braketimer.stopTimer();
        }
        if(braketimer.getTimeSeconds() > 1){
            return true;
        }
        return false;
    }

    public double[] getLeftSlideData(){
        double leftslidePos = leftslides.getCurrentPosition();
        double leftslideVelo = leftslides.getVelocity();
        return new double[] {leftslidePos,leftslideVelo};
    }

    public double[] getRightSlideData() {
        double rightslidePos = rightslides.getCurrentPosition();
        double rightslideVelo = rightslides.getVelocity();
        return new double[] {rightslidePos,rightslideVelo};
    }

    public void stopSlides(){
        leftslides.setPower(0.0);
        rightslides.setPower(0.0);
    }

    public void stopandreset(){
        leftslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        rightslides.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

    public void runslides(double power){
        rightslides.setPower(power);
        leftslides.setPower(power);
    }

    public void liftrunencoders(){
        leftslides.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
        rightslides.setMode((DcMotor.RunMode.RUN_USING_ENCODER));

    }

    public void MoveLift(double power, double inches, long secondsToWait) {
        // to go up go to 43 inches
        double ticksPerInch = (1667 / 44.75); // need to change this value when the robot is built
        int ticksToMove = (int) (inches * ticksPerInch);

        leftslides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightslides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftslides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightslides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        long endTime = System.currentTimeMillis() + secondsToWait * 1000;

        while(ticksToMove < Math.abs(leftslides.getCurrentPosition())){
            leftslides.setPower(power);
            rightslides.setPower(power);
            telemetry.addLine("slides going up");
            telemetry.update();
        }

        leftslides.setPower(0.0);
        rightslides.setPower(0.0);

    }
    public void setTargetPosition(int targetPosition, double power) {
        leftslides.setTargetPosition(targetPosition);
        rightslides.setTargetPosition(targetPosition);
        rightslides.setPower(power);
        leftslides.setPower(power);
    }

    public boolean isAtTarget() {
        return (!leftslides.isBusy() && !rightslides.isBusy());
    }

    public void stop() {
        stopSlides();
    }

    public void setLiftsCoast(){
        rightslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftslides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

}

