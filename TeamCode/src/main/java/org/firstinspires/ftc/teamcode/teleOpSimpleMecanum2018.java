package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 *import com.qualcomm.robotcore.hardware.DcMotor;
 *
 *
 */

/**
 *
 *Notes For this TeleOp Code. This code is for Comp and all proggramers should review over this
 *code and understand this code for the possibility that a question may be asked related to TeleOp and
 *you should be able to explain in good detail everything in this code.
 *11/16/17-> Changed all gamepad's in code to correct gamepad (i.e some gamepad1's to gamepad2)
 ***11/18/17-> Competition Notes below
 *Notes-> Autonomous is incorrect, Not much was wrong from a software sandpoint but hardware issues were fixed
 *Autonomous issues included: Incorrect spinning causing us to move out of destination,
 *To much time on the down motion of the clamp and arm.
 *These issues are still not resolved
 * Recomendation for autonomous issues(Not Offical):Fine tune the timer on the clamp
 * Fine tune the movements and LOWER the TIME OF MOVEMENT in autonomous.
 * List of issues at Comp(1)-> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1r_liipKBU7GHfONdxq9E6d4f7zikcCuXwDL2bsQfwm0/edit?usp=sharing
 *G-Sheet of time VS Heading for autonomous -> https://docs.google.com/a/stjoebears.com/spreadsheets/d/1pqv0iN94fFd5KvX1YIWP7z39HgpURXsscn0zPujs1q4/edit?usp=sharing
*/
@TeleOp(name="Simple Mecanum Drive 2", group="TeleOp")

public class teleOpSimpleMecanum2018 extends LinearOpMode {

    HardwareJoeBot2018 robot = new HardwareJoeBot2018();

    DcMotor  liftMotor;
    DcMotor  mainBucketMotor;
    DcMotor  intakeMotor;
    Servo liftbucket;
    Servo rightpos;
    Servo leftpos;

    double  liftpower;
    double mainpower;
    double intakepower;
    boolean bCurrStateLbump;
    boolean bPrevStateLbump;
    boolean LBPon;
    boolean bCurrStateB;
    boolean bPrevStateB;
    boolean bIntakeOn;
    boolean bCurrStateC;
    boolean bPrevStateC;
    boolean CIntakeOn;

    double forward;
    double clockwise;
    double right;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);




        waitForStart();



        //start of loop
        while (opModeIsActive()) {


            //Drive Via "Analog Sticks" (Not Toggle)
            //Set initial motion parameters to Gamepad1 Inputs
            forward = -gamepad1.left_stick_y;
            //right = gamepad1.left_stick_x;
            right = -gamepad1.left_trigger + gamepad1.right_trigger;
            clockwise = gamepad1.right_stick_x;


            // Map "power" variable to gamepad input
            mainpower = gamepad2.left_stick_y;
            mainBucketMotor.setPower(mainpower);

            liftpower = gamepad2.right_stick_y;
            liftMotor.setPower(liftpower);


//-----------------------------------------------//


            // check the status of the left bumper button on  gamepad2.
            bCurrStateLbump = gamepad2.left_bumper;

            // check for button state transitions.
            if ((bCurrStateLbump == true) && (bCurrStateLbump != bPrevStateLbump)) {
                LBPon = !LBPon;
            }
            bPrevStateLbump = bCurrStateLbump;

            if (LBPon == true) {
                liftbucket.setPosition(.18);
            } else {
                liftbucket.setPosition(.48);
            }



//--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateB = gamepad2.a;

            // check for button state transitions.
            if ((bCurrStateB == true) && (bCurrStateB != bPrevStateB)) {

                bIntakeOn = !bIntakeOn;

            }
            bPrevStateB = bCurrStateB;

            if (bIntakeOn == true) {
                intakeMotor.setPower(.45);
            } else {
                intakeMotor.setPower(0);

            }

//--------------------------------------------------------------------------------------//
            //--------------------------------------------------------------------------------------//

            // Toggle Intake  On/Off

            bCurrStateC = gamepad2.y;

            // check for button state transitions.
            if ((bCurrStateC == true) && (bCurrStateC != bPrevStateC)) {

                CIntakeOn = !CIntakeOn;

            }
            bPrevStateC = bCurrStateC;

            if (CIntakeOn == true) {
                rightpos.setPosition(0.7);
                leftpos.setPosition(0.2);
            } else {
                rightpos.setPosition(1);
                leftpos.setPosition(0);
            }

//--------------------------------------------------------------------------------------//

            // Display the current value

            telemetry.addData("Lift Bucket Motor Power", "%5.2f", liftpower);
            telemetry.addData("Main Bucket Motor Power", "%5.2f", mainpower);
            telemetry.addData("position", "%5.2f", liftbucket.getPosition());
            telemetry.addData("right_position", "%5.2f", rightpos.getPosition());
            telemetry.addData("left_position", "%5.2f", leftpos.getPosition());

            // telemetry.addData("Lift Bucket Servo", "%5.2f", liftbucketpos);


            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            idle();


        }//end while
    }
}