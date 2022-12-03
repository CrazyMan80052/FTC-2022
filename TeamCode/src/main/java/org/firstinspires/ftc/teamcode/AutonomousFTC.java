package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Autonomous Sahas")
public class AutonomousFTC extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HardwareForBot robot = new HardwareForBot();

    DcMotor leftBack = null;
    DcMotor leftFront = null;
    DcMotor rightBack = null;
    DcMotor rightFront = null;

    DcMotor liftMotor = null;

    static final double FORWARD_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
/*
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/


        waitForStart();
        forward(609.6, .1);
        //backward(200, .1);
        //strafeLeft(200, .1);
        //strafeRight(200, .1);

        // position one
            //strafeLeft(609.6);
            //forward(304.8);

        // position two
        /*
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int ticks = mmToTick(5000000);

        telemetry.addData("Ticks", ticks);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(ticks);
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);

      /*  rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
           // forward(133.35);
            //telemetry.addData("")
        // position three
            //  strafeRight(609.6);
            // forward(304.8);

/*
        leftBack.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftFront.setPower(FORWARD_SPEED);
        rightBack.setPower(FORWARD_SPEED);

        // ScanCone autoMove = new ScanCone();
        // autoMove.positionTwo();


        // Drive forward for 1 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/
    }


    public void initTickMotor(int ticks, int lf, int rf, int lb, int rb, double power) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        rightFront.setTargetPosition(-rf*ticks);
        leftFront.setTargetPosition(lf*ticks);
        leftBack.setTargetPosition(-lb*ticks);
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
            telemetry.addData("gang gang: ", "leftFRONT " + leftFront.getCurrentPosition());
            telemetry.addData("gang gang: ", "rightFRONT " + rightFront.getCurrentPosition());
            telemetry.addData("gang gang: ", "leftBack " + leftBack.getCurrentPosition());
            telemetry.addData("gang gang: ", "rightBack " + rightBack.getCurrentPosition());
            telemetry.addData("power gang: ", "leftFRONT " + leftFront.getPower());
            telemetry.addData("power gang: ", "rightFRONT " + rightFront.getPower());
            telemetry.addData("power gang: ", "leftBack " + leftBack.getPower());
            telemetry.addData("power gang: ", "rightBack " + rightBack.getPower());
            telemetry.addData("targetPositonGang: ", "PositionLF " + leftFront.getTargetPosition());
            telemetry.addData("targetPositonGang: ", "PositionRF " + rightFront.getTargetPosition());
            telemetry.addData("targetPositonGang: ", "PositionLB " + leftBack.getTargetPosition());
            telemetry.addData("targetPositonGang: ", "PositionRB " + rightBack.getTargetPosition());

            //telemetry.addData("Gang gang: ", "leftFRONT , RIGHTF , leftBACK , rightBACK ", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            //telemetry.addData("targetPositonGang: ", "leftFRONT "+ leftFront.getTargetPosition(), "RIGHTF " +  rightFront.getTargetPosition(), "leftBACK" +   leftBack.getTargetPosition(), "rightBACK " + rightBack.getTargetPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(1000); //need to be changed
    }


    public void backward(double mm, double power) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, -1, -1, 1, -1, power);
    }

    public void forward(double mm, double power) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, 1, 1, -1, 1, power);
    }

    public void strafeLeft(double mm, double power) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, -1, 1, -1, -1, power);
    }

    public void strafeRight(double mm, double power) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, 1, -1, 1, 1, power);
    }

    //diameter = 16.25mm
    //23.6184615385tick/mm
    //13in 330.2mm 23in 584.2mm 33in 838.2mm
    public void lift(int height) {
        if(height == 1) {
            liftTicks(mmToTick(330.2),0.5);
        }
        if(height == 2) {
            liftTicks(mmToTick(584.2), 0.5);
        }
        if(height == 3) {
            liftTicks(mmToTick(838.2), 0.5);
        }
    }

    public void liftTicks(int ticks, double power) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(ticks);
        liftMotor.setPower(power);
    }

    //conversion milimeters to ticks
    public int mmToTick(double mm) {
        return (int)(mm * 1.2726) * 2;
    }

}
