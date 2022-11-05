// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="newPID", group="Exercises")
@Disabled
public class newPID extends LinearOpMode
{
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    HardwareForbot robot = new HardwareForbot();
    private ElapsedTime runtime = new ElapsedTime();

    double                  globalAngle, correction, prevError=0, drivePowerCorrection=0,
                            pError, dError, error, prevTime = 0;
    double                  minPow = .1;
    double                  maxPow;

    public static double         Kpdrive    = .004;
    public static double         Kprotate   = .0003;
    public static double         Kd         = 0;
    public static double         gainforCorrection       = .0045;
    public static double         targetdistance = 35;
    public static double         increment      = .1;

    static final double COUNTS_PER_MOTOR_GOBILDA = 537.6;    // eg: Gobilda motor encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.94;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    OpenCvInternalCamera phoneCam;
    AutonomousPID.NumRingDeterminatorPipeline pipeline;

    static PIDFCoefficients velocityPIDCoeff = new PIDFCoefficients(300,0,.3,0);

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new AutonomousPID.NumRingDeterminatorPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }



        AutonomousPID.NumRingDeterminatorPipeline.RingCount position = pipeline.position;
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();


        waitForStart();


        while (opModeIsActive())
        {
            //4/24/2021 changes back to top goal shots
            sleep(5000);
            sleep(5000);
            robot.intakeMotor.setPower(-1);
            sleep(5000);


            robot.intakeMotor.setPower(0);

            /**
            robot.discShooter.setVelocity(3.87, AngleUnit.RADIANS);
            rotate(20,.3);
            //robot.discShooter.setVelocity(3.9,AngleUnit.RADIANS);
            sleep(2000);
            robot.fourthRow.setPower(1);
            sleep(1000);
            rotate(1,.3);
            //robot.discShooter.setVelocity(227, AngleUnit.DEGREES);
            //0.053 original on 2/26
            // rotate(-57.75, 0.5); //-22
            sleep(2500);
            robot.intakeMotor.setPower(-1);
            sleep(300);
            rotate(1,.3);
            //sleep(6000);
            //robot.discShooter.setVelocity(3.9,AngleUnit.RADIANS);
            //robot.discShooter.setVelocity(228, AngleUnit.DEGREES);
            sleep(5000);
            // rotate(55.75, 0.5); //20
            // sleep(3000);

            robot.intakeMotor.setPower(0);
            robot.fourthRow.setPower(0);
            robot.discShooter.setPower(0);
            rotate(-22, .3);
            sleep(1000);
            */

            /**
            robot.discShooter.setVelocity(3.9, AngleUnit.RADIANS);
            rotate(30,.2);
            sleep(2000);
            robot.fourthRow.setPower(1);
            robot.intakeMotor.setPower(-1);
            sleep(100);
            robot.fourthRow.setPower(0);
            robot.intakeMotor.setPower(0);
            rotate(-3,.2);
            robot.discShooter.setVelocity(1900);
            robot.fourthRow.setPower(1);
            robot.intakeMotor.setPower(-1);
            sleep(100);
            robot.fourthRow.setPower(0);
            robot.intakeMotor.setPower(0);
            rotate(-20,.2);
            robot.fourthRow.setPower(1);
            robot.intakeMotor.setPower(-1);
            sleep(30000);
            **/

            // move forward, right strafe, and drop the wobble goal
            // after dropping, move back, left strafe, and forward to park on the line
            encoderDrivewithPID(-84,0.5, 10);
            //rotate(-40, 0.5);
            sleep(1000);
            mecanumdrive(-27,10,.5);
            //rotate(90, .5);
            sleep(2000);
            sleep(1000);
            encoderDrivewithPID(-5, 1, 10);
            mecanumdrive(14, 10, 0.5); //left strafe
            encoderDrivewithPID(8,10, 0.5); //backward
            sleep(9000);

            /**
            sleep(2000);
            mecanumdrive(-27, 10, 0.5);
            sleep(1000);
            robot.wobbleArm.setPower(0.4);
            sleep(2000);
            robot.wobbleFinger1.setPosition(1);
            robot.wobbleFinger1.setPosition(0); //drops wobble goal
            sleep(3000);
            encoderDrivewithPID(-5, 1, 10);
            // sleep(1000);
            mecanumdrive(16, 10, 0.5); //left strafe
            encoderDrivewithPID(8,10, 0.5); //backward
            sleep(30000);
             */
        }

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double differenceAngle = angles.firstAngle - lastAngles.firstAngle;

        //makes sure value is in between -180 and 180
        if (differenceAngle < -180)
            differenceAngle += 360;
        else if (differenceAngle > 180)
            differenceAngle -= 360;

        globalAngle += differenceAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection(){
        double angle, gain = gainforCorrection, correction; //gain is how sensitive our imu is to changes

        angle = getAngle();

        if(angle==0){
            correction=0; //makes no correction to power of wheels
        }
        else{
            correction = -angle; //negative so that it correlates to the direction of left and right
        }

        correction = correction*gain;

        return correction;
    }

    public void encoderDrivewithPID(double inches, double power, double timeS){
        resetAngle();

        int driveTarget;
        double rampSpeed = minPow;

        if(opModeIsActive()){
            //currentGyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //correctionWhileRun = checkDirectionContin();

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveTarget = robot.leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            //drivePowerCorrection = pidCalculations(Kpdrive,0,Kd,inches);


            //urnError = targetAngle - currentGyroAngle.firstAngle;
            //turnPowerCorrection = Kprotate*turnError;


            robot.leftBack.setTargetPosition(driveTarget);
            robot.rightBack.setTargetPosition(driveTarget);
            robot.leftFront.setTargetPosition(driveTarget);
            robot.rightFront.setTargetPosition(driveTarget);
            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            robot.leftBack.setPower(Math.abs(.1));
            robot.rightBack.setPower(Math.abs(.1));
            robot.leftFront.setPower(Math.abs(.1));
            robot.rightFront.setPower(Math.abs(.1));


            while (opModeIsActive() && (runtime.seconds() < timeS) && (robot.leftFront.isBusy() && robot.leftBack.isBusy())) {


                drivePowerCorrection = pidCalculations(Kpdrive,0,Kd,inches);
                correction = checkDirection();

                maxPow = Range.clip(drivePowerCorrection,-power,power);


                robot.leftBack.setPower(Math.abs(drivePowerCorrection - correction));
                robot.rightFront.setPower(Math.abs(drivePowerCorrection + correction));
                robot.leftFront.setPower(Math.abs(drivePowerCorrection - correction));
                robot.rightBack.setPower(Math.abs(drivePowerCorrection + correction));




            }

            // Stop all motion;
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            /*
            robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

            sleep(250);

            resetAngle();
        }
    }

    public void rotate(double degrees, double power){
        double leftPower, rightPower;
        resetAngle();

        double current = getAngle();

        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(opModeIsActive()){
            //- degrees is right turn and + is left turn
            if(degrees<0){
                leftPower = power;
                rightPower = -power;
            }

            else if(degrees>0){
                leftPower = -power;
                rightPower = power;
            }

            else return;

            robot.leftBack.setPower(leftPower);
            robot.rightFront.setPower(rightPower);
            robot.leftFront.setPower(leftPower);
            robot.rightBack.setPower(rightPower);


            if(degrees<0){
                while(opModeIsActive() && getAngle()==0) {}

                while(opModeIsActive() && (getAngle()-current)>degrees) {}
            }
            else {
                while (opModeIsActive() && (getAngle()-current) < degrees) ;
            }



            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);


            sleep(250);

            resetAngle();
        }
    }


    public double pidCalculations(double Kp, double Ki, double Kd, double input){
        double deltaTime=0;
        ElapsedTime currentTime = new ElapsedTime();

        int driveTarget;

        currentTime.reset();

        driveTarget = (int) (input * COUNTS_PER_INCH);

        deltaTime = currentTime.seconds() - prevTime;

        error = driveTarget - robot.leftBack.getCurrentPosition();

        pError = Kp * error;

        telemetry.addData("Kp", "%f", pError);
        telemetry.update();

        dError = Kd * ((error - prevError) / deltaTime);
        drivePowerCorrection = pError + dError;

        prevError = error;
        prevTime = currentTime.seconds();

        return drivePowerCorrection;
    }

    public void mecanumdrive(double inches, double timeoutS, double power) {
        int newTarget;

        resetAngle();
        double rampSpeed = minPow;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = robot.leftBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);


            robot.leftBack.setTargetPosition(-newTarget);
            robot.rightBack.setTargetPosition(newTarget);
            robot.leftFront.setTargetPosition(newTarget);
            robot.rightFront.setTargetPosition(-newTarget);
            // Turn On RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBack.setPower(Math.abs(power)); //change these
            robot.rightBack.setPower(Math.abs(power));
            robot.rightFront.setPower(Math.abs(power));
            robot.leftFront.setPower(Math.abs(power));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.leftBack.isBusy() && robot.rightBack.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", robot.leftBack.getCurrentPosition(), robot.rightBack.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);


        }
    }

    public static class NumRingDeterminatorPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingCount {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(130, 98);
        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING = 139;
        final int ONE_RING = 131;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile AutonomousPID.NumRingDeterminatorPipeline.RingCount position = AutonomousPID.NumRingDeterminatorPipeline.RingCount.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            //position = RingCount.FOUR;

            if (avg1 > FOUR_RING) {
                position = AutonomousPID.NumRingDeterminatorPipeline.RingCount.FOUR; // Record our analysis
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (avg1 > ONE_RING) {
                position = AutonomousPID.NumRingDeterminatorPipeline.RingCount.ONE; // Record our analysis
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else {
                position = AutonomousPID.NumRingDeterminatorPipeline.RingCount.NONE; // Record our analysis
            }

            /*
             * Simply a visual aid. Serves no functional purpose.
             */


            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public int getAnalysis() {
            return avg1;
        }
    }


}