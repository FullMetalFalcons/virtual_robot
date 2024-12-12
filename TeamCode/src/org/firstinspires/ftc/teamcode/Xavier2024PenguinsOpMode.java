package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Xavier Penguins 2024 OpMode", group = "Xavier")
public class Xavier2024PenguinsOpMode extends LinearOpMode {
    public static class PARAMS {
        // drive motor setup
        public String leftFrontDriveName = "leftFront";
        public String leftBackDriveName = "leftBack";
        public String rightFrontDriveName = "rightFront";
        public String rightBackDriveName = "rightBack";

        public DcMotorSimple.Direction leftFrontDriveDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction leftBackDriveDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction rightFrontDriveDirection = DcMotorSimple.Direction.FORWARD;
        public DcMotorSimple.Direction rightBackDriveDirection = DcMotorSimple.Direction.FORWARD;
    }
    PARAMS DRIVE_PARAMS = new PARAMS();

    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide, Hanger;
    Servo Claw;

    // Encoder storage variables
    int slideLength = 0;
    final double inPerSlideTick = 0.0;

    int armAngle = 0;
    final double degreePerArmTick = 0.0;

    int currentRobotLength = 0;
    final int MAX_ROBOT_LENGTH = 42;

    public void runOpMode() {

        //Define those motors and stuff
        //The string should be the name on the Driver Hub
        m1 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftFrontDriveName);
        m2 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightFrontDriveName);
        m3 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.leftBackDriveName);
        m4 = (DcMotorEx) hardwareMap.dcMotor.get(DRIVE_PARAMS.rightBackDriveName);
        Arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        Slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        Hanger = (DcMotorEx) hardwareMap.dcMotor.get("linearActuator");

        Claw = (Servo) hardwareMap.servo.get("claw");

        //Set them to the correct modes
        //This reverses the motor direction
        m1.setDirection(DRIVE_PARAMS.leftFrontDriveDirection);
        m2.setDirection(DRIVE_PARAMS.rightFrontDriveDirection);
        m3.setDirection(DRIVE_PARAMS.leftBackDriveDirection);
        m4.setDirection(DRIVE_PARAMS.rightBackDriveDirection);

        //Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Hanger.setDirection(DcMotorSimple.Direction.REVERSE);


        //This resets the encoder values when the code is initialized
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //This makes the wheels tense up and stay in position when it is not moving, opposite is FLOAT
        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //This lets you look at encoder values while the OpMode is active
        //If you have a STOP_AND_RESET_ENCODER, make sure to put this below it
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()) {
            // Mecanum drive code
            double px = 0.0;
            double py = 0.0;
            double pa = 0.0;
            if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
                // This allows for driving via dpad as well
                // Uses ternaries to make the code more compact (condition ? if true : if false)
                px = gamepad1.dpad_left ? -0.8 : 0.0;
                px = gamepad1.dpad_right ? 0.8 : px;
                py = gamepad1.dpad_down ? -0.8 : 0.0;
                py = gamepad1.dpad_up ? 0.8 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = -gamepad1.right_stick_x;
            }
            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;
            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));
            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;
            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);


            // Arm Code
            if (gamepad1.right_bumper) {
                // Arm Up, if the limit will not be passed
                if (getNewRobotLength(0,5)) {
                    Arm.setPower(1);
                }
            } else if (gamepad1.right_trigger > 0) {
                // Arm Down, if the limit will not be passed
                if (getNewRobotLength(0,-5)) {
                    Arm.setPower(-1);
                }
            } else {
                // At Rest
                Arm.setPower(0);
            }



            // Slide Code
            if (gamepad1.left_bumper) {
                // Slide Out, if the limit will not be passed
                if (getNewRobotLength(5,0)) {
                    Slide.setPower(1);
                }
            } else if (gamepad1.left_trigger > 0) {
                // Slide In
                // I don't think a limit check is ever necessary here, but just in case...
                if (getNewRobotLength(-5,0)) {
                    Slide.setPower(-1);
                }
            } else {
                // At Rest
                Slide.setPower(0);
            }



            // Hanging Arm Code
            if (gamepad2.left_bumper) {
                // Actuator Out
                Hanger.setPower(1);
            } else {
                /* Add:
                   Hanger.getCurrentPosition() < -500 ||
                   to make the arm auto retract
                 */
                if (gamepad2.left_trigger > 0) {
                    // Retract if out
                    Hanger.setPower(-1);
                } else {
                    // Stop if fully back
                    Hanger.setPower(0);
                }
            }

            // Claw Code
            if (gamepad1.y) {
                // Open Position
                Claw.setPosition(0.3);
            } else {
                // Closed Position
                Claw.setPosition(0.6);
            }


            //if (gamepad1.a) {
                // Encoder telemetry
                telemetry.addData("Arm Pos", Arm.getCurrentPosition());
                telemetry.addData("Slide Pos", Slide.getCurrentPosition());
                telemetry.addData("Arm Power", Arm.getPower());
                telemetry.addData("Slide Power", Slide.getPower());

                telemetry.addData("Can Move", getNewRobotLength(-5,0));
            //}

            telemetry.update();

        } // opModeActive loop ends
    }

    // Method to check whether the robot will still be within size constraints after the desired movements
    public boolean getNewRobotLength(int deltaLength, int deltaAngle) {
        slideLength = (int) (Slide.getCurrentPosition() * inPerSlideTick);
        slideLength += deltaLength;

        armAngle = (int) (Arm.getCurrentPosition() * degreePerArmTick);
        armAngle += deltaAngle;

        currentRobotLength = (int) (Math.cos(armAngle) * slideLength);
        return currentRobotLength < MAX_ROBOT_LENGTH;
    }


} // end class