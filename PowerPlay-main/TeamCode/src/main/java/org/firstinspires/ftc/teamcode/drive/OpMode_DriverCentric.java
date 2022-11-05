package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class OpMode_DriverCentric extends LinearOpMode {
    public enum MoveState {
        MOVE_START,
        DROP_FIRST,
        DROP_SECOND,
        DROP_THIRD,
        LIFT_DUMP,
        RETRACT
    }

    MoveState moveState = MoveState.MOVE_START;
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    HardwareForTools tools = new HardwareForTools();

    ElapsedTime liftTimer = new ElapsedTime();

    final int STARTING_POS = -10;
    final int LOW_LEVEL = -960;
    final int MID_LEVEL = -1487;
    final int HIGH_LEVEL = -2020;
    final int DUMP_TIME = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        double intakePower;
        double cappingPower;
        telemetry.addData("Status", "Initialized");

        tools.init(hardwareMap);

        liftTimer.reset();

        tools.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);
//        drive.setPoseEstimate(new Pose2d(PoseStorage.currentPose.getX(),
//                PoseStorage.currentPose.getY(), PoseStorage.currentPose.getHeading()+360));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

//            telemetry.addData("Current State: ", moveState);
//            telemetry.addData("Time: ", liftTimer.seconds());
//            telemetry.addData("current position", tools.liftMotor.getCurrentPosition());
//            telemetry.update();

            switch (moveState){
                case MOVE_START:

                    if (gamepad2.y){
                        tools.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        tools.liftMotor.setTargetPosition(LOW_LEVEL);
                        tools.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        tools.liftMotor.setPower(1);
                        moveState = MoveState.DROP_FIRST;
                    }

                    break;

                case DROP_FIRST:
                    if(tools.liftMotor.getCurrentPosition() - LOW_LEVEL < 10){
                        if(gamepad2.y){
                            tools.liftMotor.setTargetPosition(MID_LEVEL);
                            moveState = MoveState.DROP_SECOND;
                        }else if(gamepad2.x){
                            tools.blockDropper.setPosition(.5);
                            liftTimer.reset();
                            moveState = MoveState.LIFT_DUMP;
                        }
                    }
                    break;

                case DROP_SECOND:
                    if (tools.liftMotor.getCurrentPosition() - MID_LEVEL < 10) {
                        if (gamepad2.y) {
                            tools.liftMotor.setTargetPosition(HIGH_LEVEL);
                            moveState = MoveState.DROP_THIRD;
                        } else if (gamepad2.x) {
                            tools.blockDropper.setPosition(.5);
                            liftTimer.reset();
                            moveState = MoveState.LIFT_DUMP;
                        }
                    }
                    break;

                case DROP_THIRD:
                    if (tools.liftMotor.getCurrentPosition() - HIGH_LEVEL < 10){
                        if(gamepad2.x){
                            tools.blockDropper.setPosition(.48);
                            liftTimer.reset();
                            moveState = MoveState.LIFT_DUMP;
                        }

                    }
                    break;

                case LIFT_DUMP:
                    if(liftTimer.seconds()>=DUMP_TIME){
                        tools.blockDropper.setPosition(.9);
                        tools.liftMotor.setTargetPosition(STARTING_POS);
                        tools.liftMotor.setPower(-1);
                        moveState = MoveState.RETRACT;
                    }
                    break;

                case RETRACT:
                    if(tools.liftMotor.getCurrentPosition() - STARTING_POS > -10){
                        moveState = MoveState.MOVE_START;
                    }
                    break;

                default:
                    moveState = MoveState.MOVE_START;

            }

            if (gamepad2.right_bumper && moveState != MoveState.MOVE_START){
                moveState = MoveState.LIFT_DUMP;
            }


            if(gamepad2.dpad_right){
                tools.carousolMotor.setPower(.8);
            }
            else if(gamepad2.dpad_left){
                tools.carousolMotor.setPower(-.8);
            }
            else if(gamepad2.dpad_up){
                tools.carousolMotor.setPower(0);
            }
            else if(gamepad2.left_trigger == 1 || gamepad1.left_trigger == 1){
                tools.capping1.setDirection(CRServo.Direction.FORWARD);
                tools.capping2.setDirection(CRServo.Direction.REVERSE);
                tools.capping1.setPower(1);
                tools.capping2.setPower(1);
            }
            else if(gamepad2.right_trigger == 1 || gamepad1.right_trigger == 1) {
                tools.capping1.setDirection(CRServo.Direction.REVERSE);
                tools.capping2.setDirection(CRServo.Direction.FORWARD);
                tools.capping1.setPower(1);
                tools.capping2.setPower(1);
            }
            else if(gamepad2.left_bumper || gamepad1.left_bumper){
                tools.capping1.setPower(0);
                tools.capping2.setPower(0);
            }

            /*else if(gamepad2.left_trigger == 1){
                for(double i=0; i < 1; i=i+0.05) {
                    tools.capping.setPosition(i);
                    //wait(1);
                }
            }
            else if(gamepad2.right_trigger == 1){
                tools.capping.setDirection(Servo.Direction.REVERSE);
                for(double i=0; i < 1; i=i+0.05) {
                    tools.capping.setPosition(i);
                    //wait(1);
                }
               }*/

            else{
                double intakeDrive = gamepad2.left_stick_y;
                intakePower = Range.clip(intakeDrive, -1,1);
                tools.intakeMotor.setPower(intakePower);

                double cappingDrive = gamepad2.right_stick_y;
                cappingPower = Range.clip(cappingDrive, -1,1);
                tools.cappingMotor.setPower(cappingPower);

            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y
            ).rotated(-poseEstimate.getHeading());
            if(!PoseStorage.red){
                input = new Vector2d(
                        -gamepad1.left_stick_x,
                        gamepad1.left_stick_y
                ).rotated(-poseEstimate.getHeading());
            }


            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}