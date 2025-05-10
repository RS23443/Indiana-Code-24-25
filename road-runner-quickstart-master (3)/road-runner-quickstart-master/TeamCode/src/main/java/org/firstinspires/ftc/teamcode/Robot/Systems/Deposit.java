package org.firstinspires.ftc.teamcode.Robot.Systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit extends SubsystemBase {

    private final Servo lof;
    private final Servo rof;
    private final Servo o_rightdiffy;
    private final Servo o_leftdiffy;
    private final Servo o_claw;

    // Servo position constants
    private static final double LOF_GRAB_POSITION = 0.12;
    private static final double ROF_GRAB_POSITION = 0.88;
    private static final double LOF_DROP_POSITION = 0.55;
    private static final double ROF_DROP_POSITION = 0.45;
    private static final double LEFT_DIFFY_GRAB = 0.4;
    private static final double RIGHT_DIFFY_GRAB = 0.6;
    private static final double LEFT_DIFFY_DROP = 0.45;
    private static final double RIGHT_DIFFY_DROP = 0.55;
    private static final double CLAW_OPEN_POSITION = 0.65;
    private static final double CLAW_CLOSE_POSITION = 0.1;

    public Deposit(final HardwareMap hardwareMap,
                             final String lofname, final String rofname,
                             final String ldiffyname, final String rdiffyname, final String oclawname) {
        lof = hardwareMap.get(Servo.class, lofname);
        rof = hardwareMap.get(Servo.class, rofname);
        o_leftdiffy = hardwareMap.get(Servo.class, ldiffyname);
        o_rightdiffy = hardwareMap.get(Servo.class, rdiffyname);
        o_claw = hardwareMap.get(Servo.class, oclawname);
    }

    public void grabpose() {
        open();
        o_rightdiffy.setPosition(RIGHT_DIFFY_GRAB);
        o_leftdiffy.setPosition(LEFT_DIFFY_GRAB);
        lof.setPosition(LOF_GRAB_POSITION);
        rof.setPosition(ROF_GRAB_POSITION);
    }

    public void dropose() {
        close();
        lof.setPosition(LOF_DROP_POSITION);
        rof.setPosition(ROF_DROP_POSITION);
        //o_rightdiffy.setPosition(RIGHT_DIFFY_DROP);
        //o_leftdiffy.setPosition(LEFT_DIFFY_DROP);
    }

    public void open() {
        o_claw.setPosition(CLAW_OPEN_POSITION);
    }

    public void close() {
        o_claw.setPosition(0.07);
    }

    public void spintospec() {
        lof.setPosition(0.7);
        rof.setPosition(0.3);
    }

    public void diffy_out(){
        o_rightdiffy.setPosition(RIGHT_DIFFY_DROP);
        o_leftdiffy.setPosition(LEFT_DIFFY_DROP);
    }

    public void diffy_in(){
        o_rightdiffy.setPosition(RIGHT_DIFFY_GRAB);
        o_leftdiffy.setPosition(LEFT_DIFFY_GRAB);
    }

    public void autoDrop(){
        lof.setPosition(0.55);
        rof.setPosition(0.45);

    }

    public void extDrop(){
        lof.setPosition(0.57);
        rof.setPosition(0.43);
    }
    public void dropTheSampleCombinedMethod(){
        close();
        //SleepCode(100);
        teleDrop();
        SleepCode(500); //was 1000
        open();
        SleepCode(500);
        teleBase();
        //grabpose();

    }

    public void dropTheSampleCombinedMethodV4(){
        close();
        //SleepCode(100);
        autoDrop();
        SleepCode(500); //was 1000
        open();
        SleepCode(200);
        teleGrabPose();
        //grabpose();

    }

    public void teleGrabPose(){
        open();
        o_rightdiffy.setPosition(RIGHT_DIFFY_GRAB);
        o_leftdiffy.setPosition(LEFT_DIFFY_GRAB);
        lof.setPosition(LOF_GRAB_POSITION);
        rof.setPosition(ROF_GRAB_POSITION);

    }

    public void grabSampleForParallelAction(){
        open();
        o_rightdiffy.setPosition(RIGHT_DIFFY_GRAB);
        o_leftdiffy.setPosition(LEFT_DIFFY_GRAB);
        lof.setPosition(LOF_GRAB_POSITION +0.03);
        rof.setPosition(ROF_GRAB_POSITION - 0.03);
    }
    public void SleepCode(long TargetSleep){
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < TargetSleep){
            //telemetry.addData("sleep for", TargetSleep);
        }
    }

    public void initPose(){
        close();
        lof.setPosition(LOF_GRAB_POSITION+0.2);
        rof.setPosition(ROF_GRAB_POSITION-0.2);
    }

    public void teleBase(){
        open();
        lof.setPosition(LOF_GRAB_POSITION+0.05); // 0.12 + 0.05
        rof.setPosition(ROF_GRAB_POSITION-0.05); // 0.88 - 0.05

    }

    public void teleDrop(){
        close();
        lof.setPosition(LOF_DROP_POSITION+0.1);
        rof.setPosition(ROF_DROP_POSITION-0.1);

    }

    public double[] ServoData(){
        double lofPos = lof.getPosition();
        double rofPos = rof.getPosition();
        double oClawPos = o_claw.getPosition();

        return new double[]{lofPos, rofPos,oClawPos} ;
    }

}
