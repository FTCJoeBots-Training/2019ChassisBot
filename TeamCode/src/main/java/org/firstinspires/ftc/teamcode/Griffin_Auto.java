package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Griffin_Auto", group = "Joe Bots Chassis")

public class Griffin_Auto extends LinearOpMode {

    HardwareJoeBot2019 robot = new HardwareJoeBot2019();
    DcMotor motor0;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

    @Override
    public void runOpMode() throws InterruptedException {
        motor0 = hardwareMap.dcMotor.get("motor_0");
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.init(hardwareMap, this);

        waitForStart();

        motor0.setPower(1);
        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);

        sleep(1000);

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);

        robot.moveInches(5, 1,2);

        motor0.setPower(1);
        motor1.setPower(-1);
        motor2.setPower(-1);
        motor3.setPower(1);

        sleep(500);

        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);


    }
}
