/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IMU", group = "BahTech")
// Comment this out to add to the OpMode list
public class IMUteste extends LinearOpMode
{
    // The IMU sensor object
    private final double kp = 0.001;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    IMU imu;

    @Override public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        FL = hardwareMap.dcMotor.get("FL");
        BL = hardwareMap.dcMotor.get("BL");
        FR = hardwareMap.dcMotor.get("FR");
        BR = hardwareMap.dcMotor.get("BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            while (!isStopRequested()) {

                telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);

                if (gamepad1.y) {
                    telemetry.addData("Yaw", "Resetting\n");
                    imu.resetYaw();
                } else {
                    telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
                }

                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
                telemetry.update();

                sleep(1000);
                turn(true, 90);
                }
            }
        }
    public void turn (boolean direction, double targetAngle) {
        final double limite = 3;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        double angle;

        if (direction) {
            angle = -targetAngle + currentAngle;
            if (angle < -180) angle += 360;
            if (angle - currentAngle > 180) currentAngle += 360;

            while (Math.abs(angle-currentAngle) > limite) {
                orientation = imu.getRobotYawPitchRollAngles();
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                if (angle - currentAngle > 180) currentAngle += 360;
                FL.setPower(0.3);
                FR.setPower(0.3);
                BL.setPower(0.3);
                BR.setPower(0.3);
                telemetry.addData("posição", currentAngle);
                telemetry.update();
            }
        } else {
            angle = targetAngle + currentAngle;
            if (angle > 180) angle -= 360;
            if (currentAngle - angle > 180) currentAngle -= 360;

            while (Math.abs(angle-currentAngle) > limite) {
                orientation = imu.getRobotYawPitchRollAngles();
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                if (currentAngle - angle > 180) currentAngle -= 360;
                FL.setPower(0.3);
                FR.setPower(0.3);
                BL.setPower(0.3);
                BR.setPower(0.3);
                telemetry.addData("posição", currentAngle);
                telemetry.update();
            }
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
