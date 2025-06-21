package pedroPathing.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Sensing.PDFL;
import pedroPathing.Robot.Sensing.Timer;
import pedroPathing.Robot.Systems.Deposit;


@Config
@TeleOp(name="PDFL + Voltage Sensor Data", group="Tests")
public class PDFLVerticalTuner extends OpMode {
    private PDFL pdfLController;

    // Tuning variables accessible via FTC Dashboard
    public static double kP = 0.008, kD = 0.003, kF = 0.133, kL = 0.1;
    public static double deadzone = 35.0;
    public static double homedConstant = 0;
    public static int target = 500;

    private DcMotorEx middleMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx topMotor;
    public VoltageSensor controlHubVoltageSensor;
    public Timer braketimer;
    public Deposit deposit;


    @Override
    public void init() {
        // Initialize the PDFL controller
        pdfLController = new PDFL(kP, kD, kF, kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);

        //configure your voltage sensor
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        // Configure motors
        middleMotor = hardwareMap.get(DcMotorEx.class, "middleslide");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomslide");
        topMotor = hardwareMap.get(DcMotorEx.class,"topslide");
        topMotor.setDirection(DcMotorEx.Direction.REVERSE);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //middleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        deposit = new Deposit(hardwareMap);

        braketimer = new Timer();


        // Configure telemetry with FTC Dashboard
            telemetry = FtcDashboard.getInstance().getTelemetry();

        deposit.setServoPosition(1, 0.3);
        deposit.setServoPosition(2, 0.7);
        deposit.setServoPosition(3, 0.35);
        deposit.setServoPosition(4, 0.63); // close
    }

    @Override
    public void loop() {
        //update voltage reading at the beginning of every loop
        double currentVoltage = controlHubVoltageSensor.getVoltage();

        // Update constants dynamically
        pdfLController.updateConstants(kP, kD, kF, kL);
        pdfLController.setDeadzone(deadzone);
        pdfLController.setHomedConstant(homedConstant);

        // Get the current positions of the motors
        //int mslidepos = middleMotor.getCurrentPosition();
        //int bslidepos = bottomMotor.getCurrentPosition();
        int tslidepos = topMotor.getCurrentPosition();

        // Calculate control signals
        //double error = target - mslidepos;
        //double error1 = target - bslidepos;
        double error2 = target - tslidepos;

        //double powerMiddle = pdfLController.run(error);
        //double powerBottom = pdfLController.run(error1);
        double powerTop = pdfLController.run(error2);

        double mvelo = middleMotor.getVelocity();
        double bvelo = bottomMotor.getVelocity();
        double tvelo = topMotor.getVelocity();

        //powerMiddle = Math.max(-1, Math.min(powerMiddle, 1));
        //powerBottom = Math.max(-1, Math.min(powerBottom, 1));
        powerTop = Math.max(-1, Math.min(powerTop, 1));

        if(currentVoltage < 11){
            braketimer.startTimer();
        } else{
            braketimer.stopTimer();
        }

        if(braketimer.getTimeSeconds() > 1){
            target = 0;
        }

        // Display the voltage on the driver station
        // Apply calculated power to motors
        middleMotor.setPower(powerTop);
        bottomMotor.setPower(powerTop);
        topMotor.setPower(powerTop);
        // Telemetry for debugging and monitoring
        telemetry.addData("Top Motor Position", tslidepos);
        //telemetry.addData("Middle Motor Position", mslidepos);
        //telemetry.addData("Bottom Motor Position", bslidepos);
        telemetry.addData("Top Motor Velocity", tvelo);
        telemetry.addData("Middle Motor Velocity", mvelo);
        telemetry.addData("Bottom Motor Velocity", bvelo);
        telemetry.addData("Target Position", target);
        telemetry.addData("Middle Motor Power", powerTop);
        telemetry.addData("Bottom Motor Power", powerTop);
        telemetry.addData("Top Motor Power", powerTop);
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.addData("kL", kL);
        telemetry.addData("Deadzone", deadzone);
        telemetry.addData("Control Hub Voltage", currentVoltage);
        telemetry.update();
    }
}
