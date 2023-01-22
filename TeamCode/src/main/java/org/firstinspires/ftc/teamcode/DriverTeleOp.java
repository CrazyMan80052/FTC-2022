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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode", group="Iterative Opmode")
public class DriverTeleOp extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor liftMotor = null;
    private Servo pincher = null; // aka intake servo
    private Servo pincherRight = null;

    double yPower;
    double xPower;
    double liftPower;
    double drive;
    double turn;

    int liftPosition;

    ElapsedTime liftTimer = new ElapsedTime();
    HardwareForBot robot = new HardwareForBot();

    double LOW_LEVEL;
    double MID_LEVEL;
    double HIGH_LEVEL;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //need to reverse the directions of the left motors cuz they are backwards
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pincher = hardwareMap.get(Servo.class, "pincher");
        pincherRight = hardwareMap.get(Servo.class, "pincherRight");

        pincher.setDirection(Servo.Direction.FORWARD);
        pincherRight.setDirection(Servo.Direction.FORWARD);

        final int STARTING_POS = liftMotor.getCurrentPosition();
        liftPosition = liftMotor.getCurrentPosition();
        liftPower = 0;

        LOW_LEVEL = STARTING_POS - 0.2;
        MID_LEVEL = STARTING_POS - 0.6;
        HIGH_LEVEL = STARTING_POS - 1;


        // Tell the driver that initialization is complete.
       // telemetry.addData("Status", "Initialized - Alana DriverTeleOp");
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

    public void allTheWay(double power)
    {
        if(power < 0)
        {
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()-1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(power > 0){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()+1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);
    }

    public void setLiftPower(double power)
    {
        if(power < 0)
        {
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()-1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(power > 0){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()+1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);
    }

    public void liftTicks(int ticks, double power) {
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(ticks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }

    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
         double mecanum;
         double diagonalPower;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        drive = gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x; // -
        mecanum = -gamepad1.left_stick_x;
        diagonalPower = Range.clip(mecanum, -1,1);
        yPower = Range.clip(drive, -1, 1);
        xPower = Range.clip(drive, -1, 1);

        // All of the drive motor telemetry data
        telemetry.addData("LEFT STICK-X", "gamepad1, leftstick-x: " + gamepad1.left_stick_x);
        telemetry.addData("LEFT STICK-Y", "gamepad1, leftstick-y: " + gamepad1.left_stick_y);

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
        //*/

        //telemetry.addData("Gang gang: ", "leftFRONT , RIGHTF , leftBACK , rightBACK ", leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
        //telemetry.addData("targetPositonGang: ", "leftFRONT "+ leftFront.getTargetPosition(), "RIGHTF " +  rightFront.getTargetPosition(), "leftBACK" +   leftBack.getTargetPosition(), "rightBACK " + rightBack.getTargetPosition());
        //telemetry.update();
        //TODO make a way for lift to stop moving once it hits the top or the bottom


        liftPosition = liftMotor.getCurrentPosition();

        telemetry.addData("Lift Ticks: ", liftPosition);
        telemetry.addData("Gamepad 2 left Stick", gamepad2.left_stick_y);
        telemetry.addData("This should be go up: ", gamepad2.left_stick_y < 0 && liftPosition > -3000);
        telemetry.addData("Lift power: ", liftMotor.getPower());
        //stick power
        liftPower = gamepad2.left_stick_y;
        liftPower = Range.clip(liftPower, -0.5, 0.5);

        if(gamepad2.left_stick_y < 0 && liftPosition > -3000) {
            liftPower = gamepad2.left_stick_y;
            liftPower = Range.clip(liftPower, -0.5, 0.5);
            liftMotor.setPower(liftPower);
        }
        if(gamepad2.left_stick_y > 0 && liftPosition < -100) {
            liftPower = gamepad2.left_stick_y;
            liftPower = Range.clip(liftPower, -0.5, 0.5);
            liftMotor.setPower(liftPower);
        }

        if(gamepad2.dpad_right) {
            liftTicks(1300, -0.5);
        }
        //middle
        if(gamepad2.dpad_up){
            liftTicks(2100, -0.5);
        }
        //High level
        if(gamepad2.dpad_left) {
            liftTicks(2900, -0.5);
        }
        // stop lift motor
        if(gamepad2.dpad_down){
            liftTicks(25, 0.5);
        }
        if(gamepad2.b) {
            liftTicks(200, -0.5);
        }

        telemetry.addData("targetPosition after Y: ", "Motor Target Position " + liftMotor.getTargetPosition());
        telemetry.addData("targetPosition after Y: ", "Motor Current Position " + liftMotor.getCurrentPosition());


        //open and close intake servo
        if(gamepad2.left_bumper) {
            pincher.setPosition(0.0);
            telemetry.addData("ServoWork", pincher.getPosition());
        }
        if(gamepad2.right_bumper){
            pincher.setPosition(1);
            telemetry.addData("ServoWork", pincher.getPosition());
        }

        //LEFT STRAFE
        if(gamepad1.left_stick_x < 0) {
            double power = gamepad1.left_stick_x;
            leftBack.setPower(-power);
            leftFront.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
        // RIGHT STRAFE
        if(gamepad1.left_stick_x > 0) {
            double power = gamepad1.left_stick_x;
            leftBack.setPower(-power);
            leftFront.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
        else{
            leftBack.setPower(-(yPower - turn - diagonalPower));
            rightFront.setPower((xPower + turn - diagonalPower));
            leftFront.setPower(-(yPower - turn + diagonalPower));
            rightBack.setPower(-(xPower + turn + diagonalPower));
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", yPower, xPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftBack.setPower(0);
        //leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        liftMotor.setPower(0);
    }

}
