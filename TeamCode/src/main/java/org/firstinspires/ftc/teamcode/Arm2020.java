package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor0 (left front)
 * motor1 (right front)
 * motor2 (left rear)
 * motor3 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class Arm2020 {
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor liftMotor = null;
    public DcMotor armMotor = null;

    // Declare Servos
    public Servo clampServo = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();

    //static final doubles
    static final double ARM_OUT_POSITION = 0;
    static final double ARM_IN_POSITION = 0;
    static final double CLAMP_CLOSED_POSITION = 0;
    static final double CLAMP_OPEN_POSITION = 0;


    /* Constructor */
    public Arm2020() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        armMotor = hwMap.dcMotor.get("armMotor");
        liftMotor = hwMap.dcMotor.get("liftMotor");

        // Set Default Motor Directions
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        armMotor.setPower(0);
        liftMotor.setPower(0);

        // Set motor modes
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        myOpMode.telemetry.addLine("initialized motor power to zero");
        myOpMode.telemetry.update();

        myOpMode.telemetry.addLine("initialized other motor power to zero");
        myOpMode.telemetry.update();

    }

    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */

    public void setMode(DcMotor.RunMode mode) {
        armMotor.setMode(mode);
        liftMotor.setMode(mode);

    }

    //extend arm to max length
    public void extendArm(){

    }

    //extend arm based on encoder values, used for autonomous
    public void encoderExtendArm(double encoderValue, double power){

    }

    //retract arm to default position
    public void retractArm(){

    }

    //retract arm based on encoder values, used for autonomous
    public void encoderRetractArm(double encoderValue, double power){

    }

    //close clamp to set encoder position to grab wobble goal
    public void closeClamp(){

    }

    //open clamp to default position
    public void openClamp(){

    }

    //if a button is held, the arm lifts
    public void liftArm(){

    }

    //if a button is held, the arm lowers
    public void lowerArm(){

    }
}