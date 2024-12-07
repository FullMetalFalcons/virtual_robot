package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "arm bot Penguins", group = "ArmBotPenguins")
public class ArmBotDemoPenguins extends LinearOpMode {
    DcMotorEx Arm;
    Servo Claw;

    // Encoder storage variables
    int slideLength = 0;
    final double inPerSlideTick = 0.0;

    int armAngle = 0;
    final double degreePerArmTick = 0.0;

    int currentRobotLength = 0;
    final int MAX_ROBOT_LENGTH = 42;

    public void runOpMode(){
        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //STEVE - Renamed from 'arm' to 'Arm' for Penguins
        Arm = (DcMotorEx)hardwareMap.dcMotor.get("arm_motor");
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //GyroSensor gyro = hardwareMap.gyroSensor.get("gyro_sensor");
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        //STEVE - Renamed from 'handServo' to 'Claw'
        Claw = hardwareMap.servo.get("claw");


        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        //gyro.init();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        //START PENGUINS CODE:
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



            //STEVE - Remove slide and hanger code for now

            // Claw Code
            if (gamepad1.y) {
                // Open Position
                Claw.setPosition(0.3);
            } else {
                // Closed Position
                Claw.setPosition(0.6);
            }


            //STEVE - Removed Pinpoint code

            //if (gamepad1.a) {
                // Encoder telemetry
                telemetry.addData("Arm Pos", Arm.getCurrentPosition());

                //STEVE - Comment out Slide
                //telemetry.addData("Slide Pos", Slide.getCurrentPosition());
            //}

            telemetry.update();

        } // opModeActive loop ends
    }

    // Method to check whether the robot will still be within size constraints after the desired movements
    public boolean getNewRobotLength(int deltaLength, int deltaAngle) {
        //STEVE - Change to 1 for now
        slideLength = (int) 1; //(Slide.getCurrentPosition() * inPerSlideTick);
        slideLength += deltaLength;

        armAngle = (int) (Arm.getCurrentPosition() * degreePerArmTick);
        armAngle += deltaAngle;

        currentRobotLength = (int) (Math.cos(armAngle) * slideLength);
        return currentRobotLength < MAX_ROBOT_LENGTH;
    }


}
