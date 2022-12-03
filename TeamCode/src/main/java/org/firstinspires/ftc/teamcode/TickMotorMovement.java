package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TickMotorMovement extends OpMode {
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

    public TickMotorMovement() {
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
        pincher.setDirection(Servo.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized - Autonomous");
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    public void initTickMotor(int ticks, int lf, int rf, int lb, int rb) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setTargetPosition(ticks);
        leftFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public void liftTicks(int ticks, double power) {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPosition(ticks);
        liftMotor.setPower(power);
    }

    public void backward(double mm){
        int ticks = mmToTick(mm);
        initTickMotor(ticks, -1, -1, 1, -1);
    }

    public void forward(double mm){
        int ticks = mmToTick(mm);
        initTickMotor(ticks, 1, 1, -1, 1);
    }

    public void strafeLeft(double mm) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, 1, 1, 1, -1);
    }

    public void strafeRight(double mm) {
        int ticks = mmToTick(mm);
        initTickMotor(ticks, -1, -1, -1, 1);
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

    //conversion milimeters to ticks
    public int mmToTick(double mm) {
        return (int) (mm * 23.6184615385);
    }
    //inches x tick/in
    //
    /*
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
    */
    //19384 ticks
}