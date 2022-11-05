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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="aprilAndFSM", group="aprilAndFSM")
//@Disabled
public class autoWithAprilAndFSM extends LinearOpMode {

    public enum MoveState {
        MOVE_START,
        MOVE_SHOOT,
        MOVE_FEED,
        MOVE_STOP
    }

    AprilTagDemo.MoveState moveState = AprilTagDemo.MoveState.MOVE_START;
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    //HardwareForTools tools = new HardwareForTools();
    //SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

    ElapsedTime intakeTimer = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(270));
        //robot.setPoseEstimate(startPose);

        //robot.init(hardwareMap);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        intakeTimer.reset();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        //camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
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



        waitForStart();

        boolean center = false;
        boolean right = false;
        boolean left = false;

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
                }

                telemetry.update();
            }
        }


        if(!center){
            ElapsedTime rightTime = new ElapsedTime();

            while(rightTime.seconds()<=3) {
                // Calling getDetectionsUpdate() will only return an object if there was a new frame
                // processed since the last time we called it. Otherwise, it will return null. This
                // enables us to only run logic when there has been a new frame, as opposed to the
                // getLatestDetections() method which will always return an object.
                ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

                telemetry.addData("Right Time: ", centerTime.seconds());
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
                    }

                    telemetry.update();
                }
            }
        }

        if(!right){
            left = true;
        }



//        while (!detected)
//        {
//
//            // Calling getDetectionsUpdate() will only return an object if there was a new frame
//            // processed since the last time we called it. Otherwise, it will return null. This
//            // enables us to only run logic when there has been a new frame, as opposed to the
//            // getLatestDetections() method which will always return an object.
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
//
//            telemetry.addData("detected: ", detected);
//            telemetry.addData("Current State: ", moveState);
//            telemetry.addData("Time: ", intakeTimer.seconds());
//            telemetry.update();
//
//            // If there's been a new frame...
//            if(detections != null)
//            {
//
//                // If we don't see any tags
//                if(detections.size() == 0)
//                {
//                    numFramesWithoutDetection++;
//
//                    // If we haven't seen a tag for a few frames, lower the decimation
//                    // so we can hopefully pick one up if we're e.g. far back
//                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
//                    {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
//                    }
//                }
//                // We do see tags!
//                else
//                {
//                    telemetry.addLine("it works!");
//                    numFramesWithoutDetection = 0;
//
//                    detected = true;
//
//                    // If the target is within 1 meter, turn on high decimation to
//                    // increase the frame rate
//                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
//                    {
//                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
//                    }
//
//                    for(AprilTagDetection detection : detections)
//                    {
//                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//                    }
//                }
//
//                telemetry.update();
//            }
//
//            sleep(20);
//        }
    }


}
