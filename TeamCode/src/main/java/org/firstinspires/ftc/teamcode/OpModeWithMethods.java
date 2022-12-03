package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class OpModeWithMethods extends OpMode
{
        // Declare OpMode members.
        private final ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftBack = null;
        private DcMotor rightBack = null;
        private DcMotor leftFront = null;
        private DcMotor rightFront = null;
        private DcMotor liftMotor = null;
        private Servo pincher = null;

        double leftPower;
        double rightPower;
        double drive;
        double turn;
        ElapsedTime liftTimer = new ElapsedTime();
        HardwareForBot robot = new HardwareForBot();

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
            liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

            pincher = hardwareMap.get(Servo.class, "pincher");
            pincher.setDirection(Servo.Direction.REVERSE);

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
        @Override
        public void loop() {

            // Setup a variable for each drive wheel to save power level for telemetry
            leftPower    = Range.clip(drive, -1, 1);
            rightPower   = Range.clip(drive, -1, 1);

            // double mecanum;
            // double diagonalPower;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            if (gamepad1.left_bumper) {
                pincher.setPosition(.5);
            }
            else if(gamepad1.right_bumper){
                pincher.setPosition(.9);
            }
            if(gamepad2.x) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setTargetPosition(19384);
                liftMotor.setPower(.5);
            }
            else if(gamepad2.y) {
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setTargetPosition(50);
                liftMotor.setPower(-.5);
            }
            else if(gamepad1.left_bumper) {
                turnCounterClockwise(leftPower, rightPower);
            }
            else if(gamepad1.right_bumper) {
                turnClockwise(leftPower, rightPower);
            }
            else if(gamepad1.left_stick_x > 0) {
                strafeRight(leftPower, rightPower);
            }
            else if(gamepad1.left_stick_x < 0) {
                strafeLeft(leftPower, rightPower);
            }
            else if(gamepad1.left_stick_y > 0) {
                forward(leftPower, rightPower);
            }
            else if(gamepad1.left_stick_y < 0) {
                backward(leftPower, rightPower);
            }

            // drive = gamepad1.left_stick_y;
            // turn  =  gamepad1.right_stick_x;
            // mecanum = -gamepad1.left_stick_x;
            // diagonalPower = Range.clip(mecanum, -1,1);

            /**
             leftBack.setPower(leftPower-turn-diagonalPower);
             rightFront.setPower(rightPower+turn-diagonalPower);
             leftFront.setPower(leftPower-turn+diagonalPower);
             rightBack.setPower(rightPower+turn+diagonalPower);

             leftBack.setPower(0);
             rightFront.setPower(0);
             leftFront.setPower(0);
             rightBack.setPower(0);
             **/

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }

        public void turnClockwise(double leftPower, double rightPower){
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftFront.setPower(leftPower);
            rightBack.setPower(rightPower);
        }

        public void turnCounterClockwise(double leftPower, double rightPower){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftFront.setPower(leftPower);
            rightBack.setPower(rightPower);
        }

        public void backward(double leftPower, double rightPower){
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftFront.setPower(leftPower);
            rightBack.setPower(rightPower);
        }

        public void forward(double leftPower, double rightPower){
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftFront.setPower(leftPower);
            rightBack.setPower(rightPower);
        }

        public void strafeLeft(double leftPower, double rightPower) {
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            leftBack.setPower(leftPower);
            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);
        }

        public void strafeRight(double leftPower, double rightPower) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);

            leftBack.setPower(leftPower);
            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);
        }

        //conversion inches to ticks
        public int inToTick(double inches) {
            return (int) (inches * 31.658); //inches x tick/in
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }

    }

