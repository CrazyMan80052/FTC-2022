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


package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group="drive")
public class blueCarousolWarehouse extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;



    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        HardwareForTools tools = new HardwareForTools();
        tools.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
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

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        boolean center = false;
        boolean right = false;
        int liftUp = -970;




        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(270));
        robot.setPoseEstimate(startPose);

        tools.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.setMsTransmissionInterval(50);

        ElapsedTime centerTime = new ElapsedTime();
        while(centerTime.seconds()<=3) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

            telemetry.addData("Center Time: ", centerTime.seconds());
            telemetry.addData("center", center);
            telemetry.update();

            // If there's been a new frame...
            if (detections != null) {

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    telemetry.addLine("it works!");
                    numFramesWithoutDetection = 0;

                    center = true;
                    liftUp = -1492;

                }

                telemetry.update();
            }
        }

        waitForStart();

        if(!center){
            ElapsedTime rightTime = new ElapsedTime();
            robot.turn(Math.toRadians(-20));

            while(rightTime.seconds()<=3) {

                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

                telemetry.addData("Right Time: ", rightTime.seconds());
                telemetry.addData("right: ", right);
                telemetry.update();

                // If there's been a new frame...
                if (detections != null) {

                    // If we don't see any tags
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else {
                        telemetry.addLine("it works!");
                        numFramesWithoutDetection = 0;

                        right = true;
                        liftUp = -2035;
                    }

                    telemetry.update();
                }
            }
        }


        int finalLiftUp = liftUp;
        if(!center){
            startPose = new Pose2d(-34, -61, Math.toRadians(250));
        }



        /** OLD VERSION block, carousel, warehouse **/
        TrajectorySequence trajOne = robot.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-20,-34, Math.toRadians(230)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(2)
                .addTemporalMarker(0, () -> {
                    tools.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tools.liftMotor.setTargetPosition(finalLiftUp);
                    tools.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    tools.liftMotor.setPower(Math.abs(1));
                })
                .addTemporalMarker(1, () -> {
                    tools.liftMotor.setPower(0);
                })
                .waitSeconds(2)
                .addTemporalMarker(4.5,() -> {
                    tools.blockDropper.setPosition(.49);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-34,-52, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-65, -50, Math.toRadians(270)))
                .forward(4,
                        SampleMecanumDrive.getVelocityConstraint(1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(10,() -> {
                    tools.carousolMotor.setPower(-.70);
                })
                .addTemporalMarker(15,() ->{
                    tools.carousolMotor.setPower(0);
                })
                .addTemporalMarker(20,() -> {
                    tools.blockDropper.setPosition(.9);
                    tools.liftMotor.setTargetPosition(-10);
                    tools.liftMotor.setPower(Math.abs(1));
                })
                .lineToLinearHeading(new Pose2d(0, -52),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(11, -71, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(45, -71, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();
        if(isStopRequested()) return;

        robot.followTrajectorySequence(trajOne);
        PoseStorage.currentPose = robot.getPoseEstimate();
    }

}

