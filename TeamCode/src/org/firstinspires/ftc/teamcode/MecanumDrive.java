package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;


        // drive motor setup
        public String leftFrontDriveName = "leftFront";
        public String leftBackDriveName = "leftBack";
        public String rightFrontDriveName = "rightFront";
        public String rightBackDriveName = "rightBack";

        public DcMotorSimple.Direction leftFrontDriveDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction leftBackDriveDirection = DcMotorSimple.Direction.REVERSE;
        public DcMotorSimple.Direction rightFrontDriveDirection = DcMotorSimple.Direction.FORWARD;
        public DcMotorSimple.Direction rightBackDriveDirection = DcMotorSimple.Direction.FORWARD;


        // drive model parameters
        public double inPerTick = 1.0; // SparkFun OTOS Note: you can probably leave this at 1
        public double lateralInPerTick = 0.71931971;
        public double trackWidthTicks = 12.5686455;

        // feedforward parameters (in tick units)
        public double kS = 2.024498;
        public double kV = 0.11;
        public double kA = 0.05;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI/2; // shared with path
        public double maxAngAccel = Math.PI/2;

        // path controller gains
        public double axialGain = 5.0;
        public double lateralGain = 8.0;
        public double headingGain = 12.0; // shared with turn
        // Was 5.0, 6.0, 40

        public double axialVelGain = 0.1;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.2; // shared with turn
    }

    public static Params PARAMS = new Params();
}
