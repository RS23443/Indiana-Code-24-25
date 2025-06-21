package pedroPathing.Autonomous.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Config
public class ServoPositions extends OpMode {
    private Servo lif;
    private Servo rif;
    private Servo rightdiffy;
    private Servo leftdiffy;
    private Servo claw;
    private Servo lof;
    private Servo rof;
    private Servo right_elbow;
    private Servo o_claw;
    @Override
    public void init() {
        lif = hardwareMap.get(Servo.class, "left_intake_flip");
        rif = hardwareMap.get(Servo.class, "right_intake_flip");
        leftdiffy = hardwareMap.get(Servo.class, "left_differential");
        rightdiffy = hardwareMap.get(Servo.class, "right_differential");
        claw = hardwareMap.get(Servo.class, "intake_claw");
        lof = hardwareMap.get(Servo.class, "left_outtake_flip");
        rof = hardwareMap.get(Servo.class, "right_outtake_flip");
        right_elbow = hardwareMap.get(Servo.class, "right_elbow");
        o_claw = hardwareMap.get(Servo.class, "outtake_claw");

        telemetry = FtcDashboard.getInstance().getTelemetry();

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            lof.setPosition(0.5);
            rof.setPosition(0.5);
            right_elbow.setPosition(1);
        }
        if(gamepad1.b){
            lof.setPosition(0.45);
            rof.setPosition(0.55);
            right_elbow.setPosition(1);

        }

        if(gamepad1.x){
            lof.setPosition(0.4);
            rof.setPosition(0.6);
            right_elbow.setPosition(1);

        }

        if(gamepad1.right_bumper){
            lof.setPosition(0.42);
            rof.setPosition(0.58);
            right_elbow.setPosition(1);
        }

        if(gamepad1.y){
            lof.setPosition(0.38);
            rof.setPosition(0.62);
            right_elbow.setPosition(1);

        }

        if(gamepad2.a){
            lif.setPosition(0.6);
            rif.setPosition(0.4);
        }

        if(gamepad2.b){
            leftdiffy.setPosition(0.2);
            rightdiffy.setPosition(0.8);
        }

        if(gamepad2.x){
            leftdiffy.setPosition(0.17);
            rightdiffy.setPosition(0.83);
        }

        if(gamepad2.y){
            leftdiffy.setPosition(0.15);
            rightdiffy.setPosition(0.85);
        }

        if(gamepad2.right_bumper){
            leftdiffy.setPosition(0.13);
            rightdiffy.setPosition(0.87);
        }

        if(gamepad2.left_bumper){
            leftdiffy.setPosition(0);
            rightdiffy.setPosition(1);
        }









        double lifpos = lif.getPosition();
        double rifpos = rif.getPosition();
        double ldpos = leftdiffy.getPosition();
        double rdpos = rightdiffy.getPosition();
        double icpos = claw.getPosition();
        double lofpos = lof.getPosition();
        double rofpos = rof.getPosition();
        double elbowpos = right_elbow.getPosition();
        double ocpos =o_claw.getPosition();

        telemetry.addData("lif pos", lifpos);
        telemetry.addData("rif pos", rifpos);
        telemetry.addData("leftdiffy pos", ldpos);
        telemetry.addData("rightdiffy pos", rdpos);
        telemetry.addData("intake claw pos", icpos);
        telemetry.addData("lof pos", lofpos);
        telemetry.addData("rof pos", rofpos);
        telemetry.addData("outtake elbow pos", elbowpos);
        telemetry.addData("outtake claw pos", ocpos);
    }
}
