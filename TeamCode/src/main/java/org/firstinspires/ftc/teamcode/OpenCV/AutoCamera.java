/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonomousFTC;
import org.firstinspires.ftc.teamcode.HardwareForBot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "La Camera")
public class AutoCamera extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    //Tag ID's for the cone sleeve
    int pos1 = 17;//left side position
    int pos2 = 18; //middle side position
    int pos3 = 19; //right side position

    AprilTagDetection tagOfInterest = null;

    //use the auto files
    AutonomousFTC autoMethods = new AutonomousFTC();
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
    public void runOpMode()
    {
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == pos1 || tag.id == pos2 || tag.id == pos3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //autoMethods.liftTicks(-50, 0.5);
        /* Actually do something useful */
        //default positon is positon 2
        if(tagOfInterest == null || tagOfInterest.id == pos2) {
            //write the code to get to the position 2 move forward
            //Position one
            closeServo();
            liftTicks(-200, 0.7);
            forward(28.3, 0.5);
            strafeRight(43.2, 0.5);
            liftTicks(-2900, 0.8);
            forward(5, 0.2);
            openServo();
            backward(3.1, 0.5);
            //liftTicks(-25,0.5);
            liftMotor.setPower(0);
            strafeLeft(44, 0.8);
        }
        else if (tagOfInterest.id == pos1){
            //write the code to reach left position/positon 1
            //telemetry.addLine("It detected position 1");
            //Position one
            closeServo();
            liftTicks(-200, 0.7);
            forward(28.3, 0.5);
            strafeRight(43.2, 0.5);
            liftTicks(-2900, 0.8);
            forward(5, 0.2);
            openServo();
            backward(3.1, 0.5);
            //liftTicks(-25,0.5);
            liftMotor.setPower(0);
            strafeLeft(73, 0.8);
        }
        else if(tagOfInterest.id == pos3) {
            //Position one
            closeServo();
            liftTicks(-200, 0.7);
            forward(28.3, 0.5);
            strafeRight(43.2, 0.5);
            liftTicks(-2900, 0.8);
            forward(5, 0.2);
            openServo();
            backward(3.1, 0.5);
            //liftTicks(-25,0.5);
            liftMotor.setPower(0);
            strafeLeft(12, 0.8);
        }
        //telemetry.update();


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
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
        pincher.setPosition(0.55);
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
