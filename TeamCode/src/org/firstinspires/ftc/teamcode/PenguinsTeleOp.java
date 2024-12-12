package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//==== CHANGES FOR VIRTUAL ROBOT =====
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//mport com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;

import java.util.Locale;

@TeleOp
@Config
public class PenguinsTeleOp extends LinearOpMode {
    //Initialize motors, servos, sensors, imus, etc.
    DcMotorEx m1, m2, m3, m4, Arm, Slide, Hanger;
    Servo Claw;
    public static MecanumDrive.Params DRIVE_PARAMS = new MecanumDrive.Params();
    public static PinpointDrive.Params PINPOINT_PARAMS = new PinpointDrive.Params();

    public static double arm_p = 5;
    public static double arm_i = 0.05;
    public static double arm_d = 0;

    public static double slide_p = 5;
    public static double slide_i = 0.05;
    public static double slide_d = 0;

    // Built-in PID loops for RUN_TO_POSITION
    public PIDFCoefficients armPID = new PIDFCoefficients(arm_p, arm_i, arm_d, 0);
    public PIDFCoefficients slidePID = new PIDFCoefficients(slide_p, slide_i, slide_d, 0);


    // Encoder storage variables for arm limits
    double slideLengthInches = 0;
    double INITIAL_SLIDE_LENGTH_INCHES = 16.0;
    final double INCHES_PER_SLIDE_TICK = 0.00830154812;

    double armAngleDeg = 0;
    final double INITIAL_ARM_ENCODER = 600;
    final double DEGREES_PER_ARM_TICK = 0.018326206475;
    final double MAX_ARM_ANGLE_DEGREES = 90.0;

    // The amount that the claw adds onto the robot's length
    double clawLengthAdditionalInches = 0.0;
    // The physical length of the claw itself (from the bottom of the viper slide)
    final double CLAW_LENGTH_INCHES = 9.0;

    double currentRobotLengthInches = 0.0;
    final double INITIAL_ROBOT_LENGTH_INCHES = 18.0;

    // The amount of added length due to the offset viper slide
    double parallelSlideOffsetInches = 0.0;
    // The distance between the center of the linear actuator and the bottom of the viper slide
    final double PARALLEL_SLIDE_DIFFERENCE_INCHES = 3.0;

    final double MAX_ROBOT_LENGTH_INCHES = 42.0;

    // Constants for how much anticipatory length/angle should be added when attempting to move a motor
    final double ABSOLUTE_DELTA_LENGTH_INCHES = 3.0;
    final double ABSOLUTE_DELTA_ANGLE_DEGREES = 7.0;


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
                px = gamepad1.dpad_left ? -0.2 : 0.0;
                px = gamepad1.dpad_right ? 0.2 : px;
                py = gamepad1.dpad_down ? -0.2 : 0.0;
                py = gamepad1.dpad_up ? 0.2 : py;
            } else {
                // If the dpad is not in use, drive via sticks
                px = gamepad1.left_stick_x;
                py = -gamepad1.left_stick_y;
                pa = -gamepad1.right_stick_x;
            }
            if (gamepad2.right_trigger > 0.0 || gamepad2.left_trigger > 0.0) {
                // Slow mode for turning
                // Currently uses GAMEPAD2 but probably will change in the future
                pa = gamepad2.right_trigger > 0.0 ? 0.5*gamepad2.right_trigger : 0.0;
                pa = gamepad2.left_trigger > 0.0 ? -0.5*gamepad2.left_trigger : pa;
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



            if (gamepad1.dpad_down) {
                Arm.setTargetPosition(0);
                Slide.setTargetPosition(0);

                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Arm.isBusy() || Slide.isBusy()) {
                    Arm.setPower(-1);
                    Slide.setPower(-1);

                    getNewRobotLength(0,0);
                    telemetry.addData("Arm Busy", Arm.isBusy());
                    telemetry.addData("Arm Angle", armAngleDeg);
                    telemetry.addData("Slide Busy", Slide.isBusy());
                    telemetry.addData("Slide Length", slideLengthInches);
                    telemetry.update();
                }
                Arm.setPower(0);
                Slide.setPower(0);

                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad1.dpad_up) {

                Arm.setTargetPosition((int) (45/DEGREES_PER_ARM_TICK - INITIAL_ARM_ENCODER));
                Slide.setTargetPosition((int) (20/INCHES_PER_SLIDE_TICK));

                armPID = Arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                slidePID = Slide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                armPID.p = arm_p;
                armPID.i = arm_i;
                armPID.d = arm_d;

                slidePID.p = slide_p;
                slidePID.i = slide_i;
                slidePID.d = slide_d;
                Arm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, armPID);
                Slide.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, slidePID);

                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (Arm.isBusy() || Slide.isBusy()) {
                    /*
                    double armP = (double) (Arm.getTargetPosition() - Arm.getCurrentPosition()) / Math.abs(Arm.getTargetPosition());
                    double slideP = (double) (Slide.getTargetPosition() - Slide.getCurrentPosition()) / Math.abs(Slide.getTargetPosition());
                    if (Math.abs(armP) > 0.3) {
                        Arm.setPower(armP);
                    } else {
                        Arm.setPower( (armP)/(Math.abs(armP)) * 0.3);
                    }
                    if (Math.abs(slideP) > 0.3) {
                        Slide.setPower(slideP);
                    } else {
                        Slide.setPower((slideP) / (Math.abs(slideP)) * 0.3);
                    }
                    */
                    Arm.setPower(1);
                    Slide.setPower(1);

                    getNewRobotLength(0,0);
                    telemetry.addData("Arm Busy", Arm.isBusy());
                    telemetry.addData("Arm Angle", armAngleDeg);
                    telemetry.addData("Slide Busy", Slide.isBusy());
                    telemetry.addData("Slide Length", slideLengthInches);
                    telemetry.update();
                }
                Arm.setPower(0);
                Slide.setPower(0);

                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }






            // Arm Code
            if (gamepad1.right_bumper && getNewRobotLength(0.0,ABSOLUTE_DELTA_ANGLE_DEGREES)) {
                // Arm Up, if the limit will not be passed
                Arm.setPower(1);
            } else if (gamepad1.right_trigger > 0 && getNewRobotLength(0.0,-ABSOLUTE_DELTA_ANGLE_DEGREES)) {
                // Arm Down, if the limit will not be passed
                Arm.setPower(-1);
            } else if (getNewRobotLength(0.0, ABSOLUTE_DELTA_ANGLE_DEGREES * gamepad2.left_stick_y)) {
                // Go by gamepad2 joystick
                Arm.setPower(gamepad2.left_stick_y);
            } else {
                // At Rest
                Arm.setPower(0.0);
            }



            // Slide Code
            if (gamepad1.left_bumper && getNewRobotLength(ABSOLUTE_DELTA_LENGTH_INCHES,0.0)) {
                // Slide Out, if the limit will not be passed
                Slide.setPower(1);
            } else if (gamepad1.left_trigger > 0 && getNewRobotLength(-ABSOLUTE_DELTA_LENGTH_INCHES,0.0)) {
                // Slide In
                // I don't think a limit check is ever necessary here, but just in case...
                Slide.setPower(-1);
            } else if (getNewRobotLength(ABSOLUTE_DELTA_LENGTH_INCHES * gamepad2.right_stick_y, 0.0)) {
                // Go by gamepad2 joystick
                Slide.setPower(gamepad2.right_stick_y);
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
            if (gamepad1.y || gamepad2.right_bumper || gamepad2.left_bumper) {
                // Open Position
                Claw.setPosition(0.3);
            } else {
                // Closed Position
                Claw.setPosition(0.6);
            }



            // EMERGENCY encoder reset sequence
            if ((gamepad1.start && gamepad1.back) || (gamepad2.start && gamepad2.back)) {
                // In case of "emergency," reset all encoders
                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine("Reset encoders");
            }


            if (gamepad1.a || gamepad2.a) {
                // Encoder telemetry
                telemetry.addData("Arm Pos", Arm.getCurrentPosition());
                telemetry.addData("Slide Pos", Slide.getCurrentPosition());

                getNewRobotLength(0,0);
                telemetry.addData("Arm Angle", armAngleDeg);
                telemetry.addData("Slide Length", slideLengthInches);
                telemetry.addData("Claw Length", clawLengthAdditionalInches);
                telemetry.addData("Robot Length", currentRobotLengthInches);

                armPID = Arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("P", armPID.p);
                telemetry.addData("I", armPID.i);
                telemetry.addData("D", armPID.d);
                telemetry.addData("F", armPID.f);
            }

            telemetry.update();

        } // opModeActive loop ends
    }

    // Method to check whether the robot will still be within size constraints after the desired movements
    public boolean getNewRobotLength(double deltaLengthInInches, double deltaAngleDeg) {
        slideLengthInches = INITIAL_SLIDE_LENGTH_INCHES + (Slide.getCurrentPosition() * INCHES_PER_SLIDE_TICK);
        slideLengthInches += deltaLengthInInches;

        armAngleDeg = (Arm.getCurrentPosition() + INITIAL_ARM_ENCODER) * DEGREES_PER_ARM_TICK;
        armAngleDeg += deltaAngleDeg;

        parallelSlideOffsetInches = Math.cos(Math.toRadians(90 - armAngleDeg)) * PARALLEL_SLIDE_DIFFERENCE_INCHES;

        currentRobotLengthInches = parallelSlideOffsetInches
                + (INITIAL_ROBOT_LENGTH_INCHES - INITIAL_SLIDE_LENGTH_INCHES)
                + (Math.cos(Math.toRadians(armAngleDeg)) * slideLengthInches);
        clawLengthAdditionalInches = Math.sin(Math.toRadians(armAngleDeg)) * CLAW_LENGTH_INCHES;
        currentRobotLengthInches += clawLengthAdditionalInches;

        if (armAngleDeg > MAX_ARM_ANGLE_DEGREES) {
            // Don't let the arm go too far such
            //   that the motor hangs out the back
            return false;
        } else {
            // Determine if the robot is within its size limit
            return (currentRobotLengthInches < MAX_ROBOT_LENGTH_INCHES);
        }
    }

} // end class