package pedroPathing.Robot.Systems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.Robot.Sensing.Timer;


public class Lifts extends SubsystemBase {
    public MultipleTelemetry telemetry;

    private final DcMotorEx bottomMotor;
    private final DcMotorEx middleMotor;
    private final DcMotorEx topMotor;
    public static double i = 0.00, p = 0.008, d = 0.003  ;
    //public static double p1 = 0.02, i1 = 0.00, d1 = 0.00;
    public static double maxticks = 1600;
    public static double maxTolerableTicks = 1500.00;
    private static double kG= 0.133;
    private final double ticks_in_degree = 145.1 / 180.0;

    private PDFL pdfLController;

    // Tuning variables accessible via FTC Dashboard
    public static double kP = 0.008, kD = 0.003, kF = 0.133, kL = 0.1;
    public static double deadzone = 25.0;
    public static double homedConstant = 30.0;
    public VoltageSensor controlHubVoltageSensor;
    public Timer braketimer;
    private int targetHeight;
    private boolean pdflEnabled = false;
    private long lastUpdateTime = 0;
    private final long updateInterval = 50; // Adjust this if needed
    private double position;
    private double velocity;
    private double voltage;



    public Lifts(final HardwareMap hardwareMap, final double voltage) {
        middleMotor = hardwareMap.get(DcMotorEx.class, "middleslide");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomslide");
        topMotor = hardwareMap.get(DcMotorEx.class,"topslide");
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        kP = kP * (12/voltage);
        kD = kD * (12/voltage);
        kF = kF * (12/voltage);
        kL = kL * (12/voltage);
        pdfLController = new PDFL(kP,kD,kF,kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);
        braketimer = new Timer();
        topMotor.setDirection(DcMotorEx.Direction.REVERSE);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void periodic(){
        position = topMotor.getCurrentPosition();
        velocity = topMotor.getVelocity();
        voltage = controlHubVoltageSensor.getVoltage();

    }
    public void joystick(Gamepad gamepad, double speed){
        middleMotor.setPower(-gamepad.left_stick_y* speed);
        bottomMotor.setPower(-gamepad.left_stick_y* speed);
        topMotor.setPower(-gamepad.left_stick_y * speed);

    }
    public void slidePID(int target) {
        if(middleMotor.getCurrentPosition() != target) {
            PIDController controllerleft = new PIDController(p, i, d);
            //PIDController controllerright= new PIDController(p1,i1,d1);

            int armPos = middleMotor.getCurrentPosition();
            int armPos1 = bottomMotor.getCurrentPosition();
            int armPos2 = topMotor.getCurrentPosition();
            double pid = controllerleft.calculate(armPos2, Math.min(target, maxTolerableTicks));
            //double pid1 = controllerright.calculate(armPos1, Math.min(target,maxTolerableTicks));
            double power = pid + kG;


            middleMotor.setPower(power);
            bottomMotor.setPower(power);
            topMotor.setPower(power);
        }
    }

    public void SetPower(double power){
        middleMotor.setPower(power);
        topMotor.setPower(power);
        bottomMotor.setPower(power);
    }

    public void slidePDFL(int target){
        if(middleMotor.getCurrentPosition() != target){

            int middleMotorPos = middleMotor.getCurrentPosition();
            int bottomMotorPos = bottomMotor.getCurrentPosition();
            int topMotorPos = topMotor.getCurrentPosition();

            // Calculate control signals
            //double error = target - middleMotorPos;
            //double error1 = target - bottomMotorPos;
            double error2 = target - topMotorPos;

            //double powerMiddle = pdfLController.run(error);
            //double powerBottom = pdfLController.run(error1);
            double powerTop = pdfLController.run(error2);

            //powerMiddle = Math.max(-1, Math.min(powerMiddle, 1));
            //powerBottom = Math.max(-1, Math.min(powerBottom, 1));
            powerTop = Math.max(-1,Math.min(powerTop,1));

            middleMotor.setPower(powerTop);
            bottomMotor.setPower(powerTop);
            topMotor.setPower(powerTop);
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

        //int middleMotorPos = middleMotor.getCurrentPosition();
        //int bottomMotorPos = bottomMotor.getCurrentPosition();
        int topMotorPos = topMotor.getCurrentPosition();

        //double errorMiddle = targetHeight - middleMotorPos;
        //double errorBottom = targetHeight - bottomMotorPos;
        double errorTop = targetHeight - topMotorPos;

        //double powerMiddle = pdfLController.run(errorMiddle);
        //double powerBottom = pdfLController.run(errorBottom);
        double powerTop = pdfLController.run(errorTop);

        //powerMiddle = Math.max(-1, Math.min(powerMiddle, 1));
        //powerBottom = Math.max(-1, Math.min(powerBottom, 1));
        powerTop = Math.max(-1, Math.min(powerTop, 1));

        middleMotor.setPower(powerTop);
        bottomMotor.setPower(powerTop);
        topMotor.setPower(powerTop);
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

    public double[] getMiddleMotorData(){
        double leftslidePos = middleMotor.getCurrentPosition();
        double leftslideVelo = middleMotor.getVelocity();
        return new double[] {leftslidePos,leftslideVelo};
    }

    public double[] getBottomMotorData() {
        double rightslidePos = bottomMotor.getCurrentPosition();
        double rightslideVelo = bottomMotor.getVelocity();
        return new double[] {rightslidePos,rightslideVelo};
    }

    public double[] getTopMotorData(){
        double topMotorPos = topMotor.getCurrentPosition();
        double topMotorVelo = topMotor.getVelocity();
        return new double[] {topMotorPos,topMotorVelo};
    }

    public void stopSlides(){
        middleMotor.setPower(0.0);
        bottomMotor.setPower(0.0);
        topMotor.setPower(0.0);
    }


    public void runslides(double power){
        bottomMotor.setPower(power);
        middleMotor.setPower(power);
        topMotor.setPower(power);
    }


    public boolean isAtTarget() {
        return (!middleMotor.isBusy() && !bottomMotor.isBusy() && !topMotor.isBusy());
    }

    public void stop() {
        stopSlides();
    }

    public void stopandreset(){
        topMotor.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
    }

}

