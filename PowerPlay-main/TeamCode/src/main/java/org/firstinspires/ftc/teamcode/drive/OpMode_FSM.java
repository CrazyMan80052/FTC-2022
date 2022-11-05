/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpModeComp", group="Iterative Opmode")
//@Disabled

public class OpMode_FSM extends OpMode
{
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

    HardwareForbot robot = new HardwareForbot();

    ElapsedTime liftTimer = new ElapsedTime();

    final int STARTING_POS = -10;
    final int LOW_LEVEL = -960;
    final int MID_LEVEL = -1487;
    final int HIGH_LEVEL = -2020;
    final int DUMP_TIME = 3;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        liftTimer.reset();

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Current State: ", moveState);
        telemetry.addData("Time: ", liftTimer.seconds());
        telemetry.addData("current position", robot.liftMotor.getCurrentPosition());
        telemetry.update();

        switch (moveState){
            case MOVE_START:

                if (gamepad2.y){
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.liftMotor.setTargetPosition(LOW_LEVEL);
                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotor.setPower(1);
                    moveState = MoveState.DROP_FIRST;
                }

                break;

            case DROP_FIRST:
                if(robot.liftMotor.getCurrentPosition() - LOW_LEVEL < 10){
                    if(gamepad2.y){
                        robot.liftMotor.setTargetPosition(MID_LEVEL);
                        moveState = MoveState.DROP_SECOND;
                    }else if(gamepad2.x){
                        robot.blockDropper.setPosition(.5);
                        liftTimer.reset();
                        moveState = MoveState.LIFT_DUMP;
                    }
                }
                break;

            case DROP_SECOND:
                if (robot.liftMotor.getCurrentPosition() - MID_LEVEL < 10) {
                    if (gamepad2.y) {
                        robot.liftMotor.setTargetPosition(HIGH_LEVEL);
                        moveState = MoveState.DROP_THIRD;
                    } else if (gamepad2.x) {
                        robot.blockDropper.setPosition(.5);
                        liftTimer.reset();
                        moveState = MoveState.LIFT_DUMP;
                    }
                }
                break;

            case DROP_THIRD:
                if (robot.liftMotor.getCurrentPosition() - HIGH_LEVEL < 10){
                    if(gamepad2.x){
                        robot.blockDropper.setPosition(.48);
                        liftTimer.reset();
                        moveState = MoveState.LIFT_DUMP;
                    }

                }
                break;

            case LIFT_DUMP:
                if(liftTimer.seconds()>=DUMP_TIME){
                    robot.blockDropper.setPosition(.9);
                    robot.liftMotor.setTargetPosition(STARTING_POS);
                    robot.liftMotor.setPower(-1);
                    moveState = MoveState.RETRACT;
                }
                break;

            case RETRACT:
                if(robot.liftMotor.getCurrentPosition() - STARTING_POS > -10){
                    moveState = MoveState.MOVE_START;
                }
                break;

            default:
                moveState = MoveState.MOVE_START;

        }

        if (gamepad2.right_bumper && moveState != MoveState.MOVE_START){
            moveState = MoveState.LIFT_DUMP;
        }


        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double intakePower;
        double diagonalPower;
        double carousolPower;
        double liftPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //Moves wobbleFinger
//        if (gamepad2.a) {
//            robot.blockDropper.setPosition(.5);
//        }
//        else if(gamepad2.b){
//            robot.blockDropper.setPosition(.9);
//        }
        if(gamepad2.dpad_right){
            robot.carousolMotor.setPower(.8);
        }
        else if(gamepad2.dpad_left){
            robot.carousolMotor.setPower(-.8);
        }
        else if(gamepad2.dpad_up){
            robot.carousolMotor.setPower(0);
        }

        else{
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double mecanum = -gamepad1.left_stick_x;
            //double carousolDrive = gamepad2.right_stick_x;
            double intakeDrive = gamepad2.left_stick_y;
            double liftDrive = -gamepad2.right_stick_y;
            leftPower    = Range.clip(drive, -1, 1);
            rightPower   = Range.clip(drive, -1, 1);
            diagonalPower = Range.clip(mecanum, -1,1);
            intakePower = Range.clip(intakeDrive, -1,1);


            // Send calculated power to wheels
            robot.leftBack.setPower(leftPower-turn-diagonalPower);
            robot.rightFront.setPower(rightPower+turn-diagonalPower);
            robot.leftFront.setPower(leftPower-turn+diagonalPower);
            robot.rightBack.setPower(rightPower+turn+diagonalPower);
            robot.intakeMotor.setPower(intakePower);


            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
        }
        // Show the elapsed game time and wheel power.
       // telemetry.addData("Status", "Run Time: " + runtime.toString());


    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

