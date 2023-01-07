package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "GoesRightPos3")
public class AutoRightPos3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HardwareForBot robot = new HardwareForBot();

    DcMotor leftBack = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor rightFront = null;

    DcMotor liftMotor = null;

    Servo pincher = null;

    public static double WHEEL_RADIUS = 1.88976378; // in
    public static double GEAR_RATIO = 0.5; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14; // in
    public static double TICKS_PER_REV = 383.6;

    static final double FORWARD_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        pincher = hardwareMap.get(Servo.class, "pincher");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        //Position one
        closeServo();
        liftTicks(-150, 0.7);
        forward(26.0, 0.5);
        strafeRight(39.9, 0.5);
        liftTicks(-2900, 0.8);
        forward(5.8, 0.2);
        //liftTicks(-2500, 0.5);
        openServo();
        backward(5.1, 0.5);
        liftTicks(-10, 0.5);
        strafeLeft(15, 0.8);

        //Position two
        //forward(609.6, 0.5);
        //telemetry.addData("")
        //position three
        //strafeRight(609.6);
        //forward(304.8);

    }


    public void initTickMotor(int ticks, int lf, int rf, int lb, int rb, double power) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition(rf*ticks);
        leftFront.setTargetPosition(lf*ticks);
        leftBack.setTargetPosition(lb*ticks);
        rightBack.setTargetPosition(rb*ticks);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        while(leftFront.isBusy()) {
            /*
            rightFront.setTargetPosition(rf * ticks);
            leftFront.setTargetPosition(lf * ticks);
            leftBack.setTargetPosition(lb * ticks);
            rightBack.setTargetPosition(rb * ticks);
            if (Math.abs(leftFront.getCurrentPosition()) >= Math.abs(leftFront.getTargetPosition())) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

             */
            telemetry.addData("gang gang: ", "leftFRONT " + leftFront.getCurrentPosition());
            telemetry.addData("gang gang: ", "rightFRONT " + rightFront.getCurrentPosition());
            telemetry.addData("gang gang: ", "leftBack " + leftBack.getCurrentPosition());
            telemetry.addData("gang gang: ", "rightBack " + rightBack.getCurrentPosition());
            telemetry.addData("power gang: ", "leftFRONT " + leftFront.getPower());
            telemetry.addData("power gang: ", "rightFRONT " + rightFront.getPower());
            telemetry.addData("power gang: ", "leftBack " + leftBack.getPower());
            telemetry.addData("power gang: ", "rightBack " + rightBack.getPower());
            telemetry.addData("targetPositionGang: ", "PositionLF " + leftFront.getTargetPosition());
            telemetry.addData("targetPositionGang: ", "PositionRF " + rightFront.getTargetPosition());
            telemetry.addData("targetPositionGang: ", "PositionLB " + leftBack.getTargetPosition());
            telemetry.addData("targetPositionGang: ", "PositionRB " + rightBack.getTargetPosition());

            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(500); //need to be changed
    }


    public void backward(double mm, double power) {
        int ticks = inchesToTicks(mm);
        initTickMotor(ticks, -1, -1, -1, -1, power);
    }

    public void forward(double mm, double power) {
        int ticks = inchesToTicks(mm);
        initTickMotor(ticks, 1, 1, 1, 1, power);
    }

    public void strafeLeft(double mm, double power) {
        int ticks = inchesToTicks(mm);
        initTickMotor(ticks, -1, 1, 1, -1, power);
    }

    public void strafeRight(double mm, double power) {
        int ticks = inchesToTicks(mm);
        initTickMotor(ticks, 1, -1, -1, 1, power);
    }

    //diameter = 16.25mm
    //23.6184615385tick/mm
    //13in 330.2mm 23in 584.2mm 33in 838.2mm
    //TODO: tune values
    public void lift(int height) {
        if(height == 1) {
            liftTicks(inchesToTicks(30.2),0.5);
        }
        if(height == 2) {
            liftTicks(inchesToTicks(30.2), 0.5);
        }
        if(height == 3) {
            liftTicks(inchesToTicks(838.2), 0.5);
        }
    }

    public void liftTicks(int ticks, double power) {
        liftMotor.setTargetPosition(ticks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        while(liftMotor.isBusy()) {
            telemetry.addData("Lift Current Pos: ", liftMotor.getCurrentPosition());
        }
        sleep(500);
    }

    //conversion inches to ticks
    public static int inchesToTicks(double in) {
        return (int) (in / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO) * TICKS_PER_REV);
    }

    //Servo Open
    public void openServo() {
        pincher.setPosition(0.00);
        sleep(2000);
    }
    public void closeServo() {
        pincher.setPosition(1.00);
        sleep(2000);
    }

    public void liftDown() {
        liftMotor.setPower(0);
    }
    //public int encoderTicksToInches(double mm) {
    //    return (int)(mm * 1.2726) * 2;
    //}

}

