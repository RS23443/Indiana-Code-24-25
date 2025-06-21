package pedroPathing.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import pedroPathing.Robot.Constants;
import pedroPathing.Robot.Sensing.Timer;
import pedroPathing.Robot.Systems.Intake;
import pedroPathing.Robot.Systems.Lifts;


@TeleOp
public class IntakeClawVoltageDetetction extends OpMode {
    public VoltageSensor controlHubVoltageSensor;
    public Intake intake;
    public Timer timer;
    public double voltage;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        timer = new Timer();
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        voltage = controlHubVoltageSensor.getVoltage();
        telemetry = FtcDashboard.getInstance().getTelemetry();


    }

    @Override
    public void loop() {
        if(gamepad1.a){
            intake.setServoPosition(6, Constants.intakeActive[4]);
            voltage = controlHubVoltageSensor.getVoltage();
        }
        if(controlHubVoltageSensor.getVoltage() < voltage - 0.1 && (controlHubVoltageSensor.getVoltage() > voltage - 0.2)){
            timer.startTimer();
            if(timer.getTimeMillis() > 500){
                telemetry.addLine("spike detected");
            }
        } else if (controlHubVoltageSensor.getVoltage() < voltage - 0.2) {
            timer.startTimer();
            if(timer.getTimeMillis() > 500){
                telemetry.addLine("blocked grabbed wrong");
            }
        }
        else {
            timer.resetTimer();
            telemetry.addLine("spike not detected");

        }


        if(gamepad1.b){
            intake.setServoPosition(6, Constants.intakeSampleReset[4]-0.02);
        }




        telemetry.addData("intake Voltage", controlHubVoltageSensor.getVoltage());
    }
}
