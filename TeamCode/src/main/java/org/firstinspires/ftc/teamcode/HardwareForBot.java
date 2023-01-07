
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.threeten.bp.DayOfWeek;

public class HardwareForBot {
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;

    public DcMotor liftMotor = null;

    public Servo pincher = null;

    public Servo pincherRight = null;
    /* *//* Constructor *//*
    public HardwareForbot(){

    }
    */
    
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;

        // Define and Initialize Motors
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");

        liftMotor = hwMap.get(DcMotor.class, "liftMotor");

        pincher = hwMap.get(Servo.class, "pincher");

        pincherRight = hwMap.get(Servo.class, "pincherRight");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD); //FORWARD
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

